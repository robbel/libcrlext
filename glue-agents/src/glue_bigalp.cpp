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

    if (argc != 3 && argc != 4) {
        LOG_ERROR("Usage: " << argv[0] << " <config.xml> <graph.dat> [SPUDD-OPTDual.ADD file]");
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

      // run the ALP planner
      _alpp = boost::make_shared<_ALPPlanner>(fmdp, 0.9);

      LOG_INFO("ALP planner planning for graphprop_" << argv[1] << "_" << argv[2] << "..");
      long start_time = time_in_milli();
      _alpp->setFactoredValueFunction(fval); // this will be computed
      int res = _alpp->plan();
      long end_time = time_in_milli();
      LOG_INFO("ALP planner returned after " << end_time - start_time << "ms");

      if(res) {
        LOG_ERROR("ALP planner failure: " << (res == 1 ? "generateLP()" : "solve()") << " failed"); // else: lp successfully generated
        return EXIT_FAILURE;
      }

      LOG_INFO("ALP planner successfully initialized.");
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

#if 0
        // compute some metrics given the optimal policy
        if(argc == 4) {
            _SpuddPolicy optpolicy(_domain, argv[3]);
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
