/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include <iostream>
#include <cassert>
#include <string.h>
#include <rlglue/Agent_common.h>
#include <rlgnmagent.h>
#include "crl/crl.hpp"
#include "crl/alp.hpp"
#include "crl/env_sysadmin.hpp"
#include "crl/glue_agent.hpp"
#include "cpputil.hpp"
#include "logger.hpp"

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
    agent = boost::make_shared<_Agent>(_alpp, learner);
  }
  catch(const cpputil::Exception& e) {
    LOG_FATAL(e);
    throw;
  }

  return agent;
}

StateMapper getStateMapper() {
    return StateMapper();
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

    if (argc != 3) {
        LOG_ERROR("Usage: " << argv[0] << " <\"star\"|\"ring\"> <computer_number>");
        return EXIT_FAILURE;
    }

    try {
        long long comp_no = std::atoll(argv[2]);
        sysadmin::Sysadmin thesys;
        if(comp_no <= 0 || !(thesys = buildSysadmin(argv[1], static_cast<Size>(comp_no)))) {
            LOG_ERROR("Instantiation of Multi-agent Sysadmin problem failed.");
            return EXIT_FAILURE;
        }

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

        // pair basis
        for(Size fa = 0; fa < ranges.size(); fa+=2) { // assumption: DBN covers all domain variables
            auto I_o = boost::make_shared<_Indicator<Reward>>(_domain, SizeVec({fa,fa+1}), State(_domain,0));
            _StateIncrementIterator sitr(I_o->getSubdomain());
            while(sitr.hasNext()) {
                auto I = boost::make_shared<_Indicator<Reward>>(_domain, SizeVec({fa,fa+1}), sitr.next());
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
        // run the ALP planner
        _alpp = boost::make_shared<_ALPPlanner>(fmdp, 0.9);

        LOG_INFO("ALP planner planning for sysadmin_" << argv[1] << "_" << argv[2] << "..");
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

    } catch(const cpputil::Exception& e) {
        LOG_ERROR(e);
        return EXIT_FAILURE;
    }

    sprintf(params, "ma-sysadmin=%s,%s", argv[1], argv[2]);
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
