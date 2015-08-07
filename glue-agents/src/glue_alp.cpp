/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include <iostream>
#include <fstream>
#include <cassert>
#include <string.h>
#include <rlglue/Agent_common.h>
#include <rlgnmagent.h>
#include "crl/crl.hpp"
#include "crl/alp.hpp"
#include "crl/basis_gen.hpp"
#include "crl/env_sysadmin.hpp"
#include "crl/glue_agent.hpp"
#include "crl/conversions.hpp"
#include "crl/vi.hpp"
#include "cpputil.hpp"
#include "logger.hpp"
#if HAS_SPUDD
#include "crl/spudd.hpp"
#endif

using namespace crl;
using namespace cpputil;

// globals
ALPPlanner _alpp;
Domain _domain;

namespace {

/// \brief Initialize the given value function with the basis conjunctions in \a basisFile
void loadConjunctiveBasis(FactoredValueFunction& fval, const string& basisFile) {
  ifstream file(basisFile);
  if (!file) {
      throw cpputil::InvalidException("File not found.");
  }
  const auto& basisVec = fval->getBasis();
  std::string line;
  while (std::getline(file, line)) {
      std::stringstream stream(line);
      string tok;
      SizeVec joint_base;
      while(std::getline(stream, tok, ',')) {
          Size basisId = std::stoull(tok);
          if(in_pos_interval(basisId, basisVec.size()-1)) {
              joint_base.push_back(basisId);
          } else {
              throw InvalidException("Invalid basis in conjunction string.");
          }
      }
      // obtain conjunctive basis
      DiscreteFunction<Reward> candf = algorithm::binconj<Reward>(joint_base, fval);
      if(!candf) { // no consistent indicator exists for h1 ^ h2
          continue;
      }
      LOG_INFO("Inserting conjunctive basis " << (*candf));
      fval->addBasisFunction(std::move(candf), 0.);
  }
}

/// \brief Initialize the given value function with the weight vector in \a weightsFile
void loadWeights(FactoredValueFunction& fval, const string& weightsFile) {
    ifstream file(weightsFile);
    if (!file) {
        throw cpputil::InvalidException("File not found.");
    }
    // assume simply that basis function layout matches provided file
    vector<double> weights;
    std::string line;
    while (std::getline(file, line)) {
        double w_i = strtod(line.c_str(), NULL); // read preserving precision
        weights.push_back(w_i);
    }
    if(weights.size() != fval->getWeight().size()) {
        throw cpputil::InvalidException("Basis function layout does not match that in weight vector file.");
    }
    // assign weights to value function
    vector<double>& vfnweights = fval->getWeight();
    std::copy(weights.begin(), weights.end(), vfnweights.begin());
}

} // anonymous ns

namespace crl {

//
// Making the agent available in rl-glue
//

Agent getCRLAgent(Domain domain) {
  assert(domain->getNumStates() == _domain->getNumStates() &&
         domain->getNumActions() == _domain->getNumActions()); // sanity check

  Agent agent;
  try {
    Learner learner;
    agent = boost::make_shared<_Agent>(domain, _alpp, learner);
  }
  catch(const cpputil::Exception& e) {
    LOG_FATAL(e);
    throw;
  }

  return agent;
}

StateMapper getStateMapper() {
    return nullptr;
}

} // namespace crl

char params[256];
const char* agent_message(const char* inMessage) {
	if (strcmp(inMessage,"id")==0)
		return (char*)"alp";
	if (strcmp(inMessage,"version")==0)
		return (char*)"1";
	if (strcmp(inMessage,"param")==0)
		return params;
	if (!strncmp(inMessage, "seed", 4)) {
		long seed = atoi(inMessage+5);
		srand(seed);
	}
	return (char*)"";
}

// launch networked rl-glue agent through rlgnm library
int main(int argc, char** argv) {
    LOG_INFO("This is the (experimental) ALP agent for currently the sysadmin environment only.");

    if (argc < 3 || argc > 13) {
        LOG_ERROR("Usage: " << argv[0] << " <\"star\"|\"ring\"> <computer_number> [-t \"simple\"] [-s SPUDD-OPTDual.ADD file] [-w Basis-weight-vector file] [-b Basis-conjunction file] [-wopt Optimal-basis-weight-vector file] [-bopt Optimal-basis-conjunction file]");
        return EXIT_FAILURE;
    }

    try {
        vector<string> remParams(argv+3, argv+argc);
        bool simple = false;
        auto tIt = std::find(remParams.begin(), remParams.end(), "-t");
        if(tIt != remParams.end() && (++tIt) != remParams.end()) {
            simple = *tIt == "simple";
        }
        long long comp_no = std::atoll(argv[2]);
        sysadmin::Sysadmin thesys;
        if(comp_no <= 0 || !(thesys = !simple ? buildSysadmin(argv[1], static_cast<Size>(comp_no)) :
                             buildSimpleSysadmin(argv[1], static_cast<Size>(comp_no)))) {
            LOG_ERROR("Instantiation of " << (simple ? "simple " : "") << "Multi-agent Sysadmin problem failed.");
            return EXIT_FAILURE;
        }
        // parse remaining arguments
        string weightsFile = "";
        string spuddFile = "";
        string basisFile = "";
        auto pIt = std::find(remParams.begin(), remParams.end(), "-w");
        if(pIt != remParams.end() && (++pIt) != remParams.end()) {
            weightsFile = *pIt;
        }
        pIt = std::find(remParams.begin(), remParams.end(), "-s");
        if(pIt != remParams.end() && (++pIt) != remParams.end()) {
            spuddFile = *pIt;
        }
        pIt = std::find(remParams.begin(), remParams.end(), "-b");
        if(pIt != remParams.end() && (++pIt) != remParams.end()) {
            basisFile = *pIt;
        }
        sprintf(params, (!simple ? "ma-sysadmin=%s,%s" : "sysadmin=%s,%s"), argv[1], argv[2]);
        //params[0] = '\0'; // the empty string

        // create planner
        _domain = thesys->getDomain();
        FactoredMDP fmdp = thesys->getFactoredMDP();
        FactoredValueFunction fval = boost::make_shared<_FactoredValueFunction>(_domain);
        LOG_INFO(fmdp->T());

        //
        // Setting basis function
        //
        const RangeVec& ranges = _domain->getStateRanges();
#if 0
        // exhaustive indicator basis
        _StateIncrementIterator sitr(_domain);
        while(sitr.hasNext()) {
            auto I = boost::make_shared<_Indicator<Reward>>(_domain, cpputil::ordered_vec<Size>(_domain->getNumStateFactors()), sitr.next());
            fval->addBasisFunction(std::move(I), 0.);
        }
#endif
#if 0
        // pair basis (inserted first for counting pair_size below)
        for(Size fa = 0; fa < ranges.size(); fa+=2) { // assumption: DBN covers all domain variables
            auto I_o = boost::make_shared<_Indicator<Reward>>(_domain, SizeVec({fa,fa+1}), State(_domain,0));
            _StateIncrementIterator sitr(I_o->getSubdomain());
            while(sitr.hasNext()) {
                auto I = boost::make_shared<_Indicator<Reward>>(_domain, SizeVec({fa,fa+1}), sitr.next());
                fval->addBasisFunction(std::move(I), 0.);
            }
        }
        auto pair_size = fval->getBasis().size();
#endif

        // individual basis
        for(Size fa = 0; fa < ranges.size(); fa++) { // assumption: DBN covers all domain variables
            auto I_o = boost::make_shared<_Indicator<Reward>>(_domain, SizeVec({fa}), State(_domain,0));
            _StateIncrementIterator sitr(I_o->getSubdomain());
            while(sitr.hasNext()) {
                auto I = boost::make_shared<_Indicator<Reward>>(_domain, SizeVec({fa}), sitr.next());
                fval->addBasisFunction(std::move(I), 0.);
            }
        }

#if 0
        // add the constant basis (to guarantee LP feasibility)
        auto cfn = boost::make_shared<_ConstantFn<Reward>>(_domain);
        fval->addBasisFunction(std::move(cfn), 0.);
        // add more basis functions
        for(Size fa = 0; fa < ranges.size(); fa+=2) { // over all load variables
          auto I = boost::make_shared<_Indicator<Reward>>(_domain);
          I->addStateFactor(fa);
          State dummy_s; // we don't care about actual domain here
          dummy_s.setIndex((Factor)sysadmin::Status::GOOD);
          I->setState(dummy_s);
          fval->addBasisFunction(std::move(I), 0.);
        }
#endif
        // add additional conjunctive features if supplied
        if(!basisFile.empty()) {
            LOG_INFO("Conjunctive basis file supplied: " << basisFile);
            loadConjunctiveBasis(fval, basisFile);
        }

        // initialize the ALP planner
        _alpp = boost::make_shared<_ALPPlanner>(fmdp, 0.9);
        _alpp->setFactoredValueFunction(fval); // this will be computed

        if(!weightsFile.empty()) {
            LOG_INFO("Basis function weight vector supplied: " << weightsFile);
            loadWeights(fval, weightsFile);
            // compute backprojections
            _alpp->precompute();

            LOG_INFO("Value function successfully initialized.");
        }
        else {
            // run the ALP planner
            LOG_INFO("ALP planner planning for sysadmin_" << argv[1] << "_" << argv[2] << "..");
            long start_time = time_in_milli();
            int res = _alpp->plan();
            long end_time = time_in_milli();
            LOG_INFO("ALP planner returned after " << end_time - start_time << "ms");

            if(res) {
                LOG_ERROR("ALP planner failure: " << (res == 1 ? "generateLP()" : "solve()") << " failed"); // else: lp successfully generated
                return EXIT_FAILURE;
            }

            LOG_INFO("ALP planner successfully initialized.");
        }
#if 0
        // L_0, L_1 norm of solution vector
        double l1w = 0., l0w = 0;
        int counter = 0, l0pair = 0, l0single = 0;
        for(auto v : fval->getWeight()) {
            LOG_INFO(" W: " << std::fixed << v);
            l1w += std::abs(v);
            if(!cpputil::approxEq(v, 0.)) {
                l0w++;
                if(counter < pair_size) {
                    l0pair++;
                }
                else {
                    l0single++;
                }
            }
            counter++;
        }
        LOG_INFO("[w norms] L_1 = " << l1w << " L_0 = " << l0w << " (pair: " << l0pair << ", single: " << l0single << ")");
#endif
//#if !NDEBUG
//        _StateIncrementIterator sitr(_domain);
//        while(sitr.hasNext()) {
//            const State& s = sitr.next();
//            auto tpl = fval->getBestAction(s);
//            LOG_DEBUG(s << ": " << std::get<0>(tpl) << " w/:" << std::get<1>(tpl));
//        }
//#endif

#if HAS_SPUDD
        // compute some metrics given the optimal policy
        if(!spuddFile.empty()) {
            LOG_INFO("SPUDD optimum policy file provided: " << spuddFile);
            _SpuddPolicy optpolicy(_domain, spuddFile.c_str());
            _StateIncrementIterator sitr(_domain);
            double Vmax = 0.;
            double linf = 0.;
            double l1   = 0.; // note that we are using uniform alphas
            while(sitr.hasNext()) {
                const State& s = sitr.next();
                tuple<Action,double> res = optpolicy.getActionValue(s);
                double optval = std::get<1>(res);
                LOG_DEBUG("Policy: " << s << " (opt): " << std::get<0>(res) << " val: " << optval << " (alp): " << fval->getV(s));
                double valdiff = std::abs(optval - fval->getV(s));
                l1 += valdiff;
                if(valdiff > linf) {
                  linf = valdiff;
                }
                if(optval > Vmax) {
                    Vmax = optval;
                }
            }
            LOG_INFO("[spudd] L_inf = " << linf << " L_inf^rel = " << linf/Vmax << " L_1 = " << l1);
#if 0
            // compute same stats for value iteration
            crl::MDP mdp = convertToMDP(fmdp);
            VIPlanner planner(new _FlatVIPlanner(_domain, mdp, .0001, 0.9));
            planner->plan();
            QTable qtable = planner->getQTable();

            sitr.reset();
            linf = 0.;
            l1 = 0.;
            while(sitr.hasNext()) {
                const State& s = sitr.next();
                double valdiff = std::abs(qtable->getV(s) - fval->getV(s));
                l1 += valdiff;
                if(valdiff > linf) {
                  linf = valdiff;
                }
            }
            LOG_INFO("[vi] L_inf = " << linf << " L_1 = " << l1);
#endif
        }
#endif
        // compute some metrics given an optimal (factored) policy
        string optWeightsFile = "";
        string optBasisFile = "";
        pIt = std::find(remParams.begin(), remParams.end(), "-wopt");
        if(pIt != remParams.end() && (++pIt) != remParams.end()) {
            optWeightsFile = *pIt;
        }
        pIt = std::find(remParams.begin(), remParams.end(), "-bopt");
        if(pIt != remParams.end() && (++pIt) != remParams.end()) {
            optBasisFile = *pIt;
        }
        if(!optWeightsFile.empty() && !optBasisFile.empty()) {
          LOG_INFO("Computing maximum error between current and optimal policies");

          // Initialize optimal value function
          FactoredValueFunction optfval = boost::make_shared<_FactoredValueFunction>(_domain);
          for(Size fa = 0; fa < ranges.size(); fa++) { // assumption: DBN covers all domain variables
              auto I_o = boost::make_shared<_Indicator<Reward>>(_domain, SizeVec({fa}), State(_domain,0));
              _StateIncrementIterator sitr(I_o->getSubdomain());
              while(sitr.hasNext()) {
                  auto I = boost::make_shared<_Indicator<Reward>>(_domain, SizeVec({fa}), sitr.next());
                  optfval->addBasisFunction(std::move(I), 0.);
              }
          }
          // load optimal value function configuration
          loadConjunctiveBasis(optfval, optBasisFile);
          loadWeights(optfval, optWeightsFile);

          // TODO: this maxDiff computation can be replaced with two maximizations over a cost network
          // Exhaustive doesn't work for larger S,A
          _StateIncrementIterator sitr(_domain);
          double Vmax = -std::numeric_limits<double>::infinity();
          double linf = 0.;
          while(sitr.hasNext()) {
              const State& s = sitr.next();
              double optval = optfval->getV(s);
              double valdiff = std::abs(fval->getV(s) - optval);
              if(valdiff > linf) {
                linf = valdiff;
              }
              if(optval > Vmax) {
                  Vmax = optval;
              }
          }
          LOG_INFO("[Abs error to V*] L_inf = " << linf);
          if(!cpputil::approxEq(Vmax, 0.)) {
             LOG_INFO("Rel error to V*] L_inf^rel = " << linf/Vmax);
          }
       }
    } catch(const cpputil::Exception& e) {
        LOG_ERROR(e);
        return EXIT_FAILURE;
    }

//    char* host = 0;
//    short port = 0;
//    if (argc == 3) {
//        host = strtok(argv[2], ":");
//        port = atoi(strtok(0, ":"));
//    }

    // run main glue agent loop
    glue_main_agent(0, 0);

    return EXIT_SUCCESS;
}
