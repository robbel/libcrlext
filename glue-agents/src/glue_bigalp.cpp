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
#include "crl/env_graphprop.hpp"
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
    LOG_INFO("This is the (experimental) ALP agent for currently the (big) GraphProp environment only.");

    if (argc < 3 || argc > 7) {
        LOG_ERROR("Usage: " << argv[0] << " <config.xml> <graph.dat> [-w Basis-weight-vector file] [-b Basis-conjunction file]");
        return EXIT_FAILURE;
    }

    try {
      ifstream iscfg(argv[1]);
      ifstream isdat(argv[2]);
      graphprop::GraphProp thegrp;
      if(!(thegrp = readGraphProp(iscfg, isdat, false))) {
        LOG_ERROR("Instantiation of Multi-agent GraphProp problem failed: error while reading from " << argv[1] << " or " << argv[2]);
        return EXIT_FAILURE;
      }
      // parse remaining arguments
      vector<string> remParams(argv+3, argv+argc);
      string weightsFile = "";
      string basisFile = "";
      auto pIt = std::find(remParams.begin(), remParams.end(), "-w");
      if(pIt != remParams.end() && (++pIt) != remParams.end()) {
          weightsFile = *pIt;
      }
      pIt = std::find(remParams.begin(), remParams.end(), "-b");
      if(pIt != remParams.end() && (++pIt) != remParams.end()) {
          basisFile = *pIt;
      }

      // create planner
      _domain = thegrp->getDomain();
      FactoredMDP fmdp = thegrp->getFactoredMDP();
      FactoredValueFunction fval = boost::make_shared<_FactoredValueFunction>(_domain);
      LOG_INFO(fmdp->T());

      //
      // Setting basis function
      //
      const RangeVec& ranges = _domain->getStateRanges();
#if 0
        // pair basis
        for(Size fa = 0; fa < ranges.size(); fa+=2) { // assumption: DBN covers all domain variables
            auto I_o = boost::make_shared<_Indicator<Reward>>(_domain, SizeVec({fa,fa+1}), State(_domain,0));
            _StateIncrementIterator sitr(I_o->getSubdomain());
            while(sitr.hasNext()) {
                auto I = boost::make_shared<_Indicator<Reward>>(_domain, SizeVec({fa,fa+1}), sitr.next());
                fval->addBasisFunction(std::move(I), 0.);
            }
        }
#endif
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

      //LOG_INFO("Variables: " << ranges.size());
      for(Size fa = 0; fa < ranges.size(); fa++) { // assumption: DBN covers all domain variables
          auto I_o = boost::make_shared<_Indicator<Reward>>(_domain, SizeVec({fa}), State(_domain,0));
          _StateIncrementIterator sitr(I_o->getSubdomain());
          while(sitr.hasNext()) {
              auto I = boost::make_shared<_Indicator<Reward>>(_domain, SizeVec({fa}), sitr.next());
              fval->addBasisFunction(std::move(I), 0.);
          }
      }
//      for(Size fa = 0; fa < 4; fa+=2) {
//          auto I_o = boost::make_shared<_Indicator<Reward>>(_domain, SizeVec({fa,fa+1}), State(_domain,0));
//          _StateIncrementIterator sitr(I_o->getSubdomain());
//          while(sitr.hasNext()) {
//              auto I = boost::make_shared<_Indicator<Reward>>(_domain, SizeVec({fa,fa+1}), sitr.next());
//              fval->addBasisFunction(std::move(I), 0.);
//          }
//      }

      // add additional conjunctive features if supplied
      if(!basisFile.empty()) {
          LOG_INFO("Conjunctive basis file supplied: " << basisFile);
          loadConjunctiveBasis(fval, basisFile);
      }

      // initialize the ALP planner
      _alpp = boost::make_shared<_ALPPlanner>(fmdp, 0.9);
      _alpp->setFactoredValueFunction(fval); // this will be computed

      if(!weightsFile.empty()) {
          LOG_INFO("Basis function weight vector supplied.");
          loadWeights(fval, weightsFile);
          // compute backprojections
          _alpp->precompute();

          LOG_INFO("Value function successfully initialized.");
      }
      else {
          // run the ALP planner
          LOG_INFO("BIGALP planner planning for graphprop_" << argv[1] << "_" << argv[2] << "..");
          long start_time = time_in_milli();
          int res = _alpp->plan();
          long end_time = time_in_milli();
          LOG_INFO("BIGALP planner returned after " << end_time - start_time << "ms");

          if(res) {
              LOG_ERROR("BIGALP planner failure: " << (res == 1 ? "generateLP()" : "solve()") << " failed"); // else: lp successfully generated
              return EXIT_FAILURE;
          }

          LOG_INFO("BIGALP planner successfully initialized.");
//#if !NDEBUG
//        for(auto v : fval->getWeight()) {
//          LOG_DEBUG(" W: " << std::fixed << v);
//        }

//        _StateIncrementIterator sitr(_domain);
//        while(sitr.hasNext()) {
//            const State& s = sitr.next();
//            auto tpl = fval->getBestAction(s);
//            LOG_DEBUG(s << ": " << std::get<0>(tpl) << " w/:" << std::get<1>(tpl));
//        }
//#endif
      }

    } catch(const cpputil::Exception& e) {
        LOG_ERROR(e);
        return EXIT_FAILURE;
    }

    sprintf(params, "graphprop=%s,%s", argv[1], argv[2]);
    //params[0] = '\0'; // the empty string

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
