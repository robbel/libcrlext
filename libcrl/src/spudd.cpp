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
#include "crl/spudd.hpp"
#include "crl/conversions.hpp"
#include "cpputil.hpp"
#include "logger.hpp"

using namespace std;
using namespace crl;

SPUDDMDP::~SPUDDMDP() {
    ::MDP::mvinput = true;
    ::MDP::allocateMemory();
    // `forgotten' in Spudd's ::MDP::init()
    Pair reward;
    reward.set(0.);
    RewardD = Cudd_addConst(gbm,&reward);
    Cudd_Ref(RewardD);
    Pair d;
    d.set(0.);
    discount = Cudd_addConst(gbm,&d);
    Cudd_Ref(discount);
    // work around broken dtor
    if (actionnames) {
      for (int i = 0; i < numactions; i++) {
        free(actionnames[i]);
        free(actionlist[i].name);
      }
      free(actionnames);
    }
    actionnames = nullptr;
    ::MDP::numactions = 0;
}

namespace crl {

_SpuddPolicy::_SpuddPolicy(const Domain& domain, const char* filename)
: _domain(domain) {
    // test if file exists
    ifstream file(filename);
    if (!file) {
        throw cpputil::InvalidException("File not found.");
    }

    // allocate memory
    _mdp = boost::make_shared<SPUDDMDP>();
    DdNode **list = new DdNode*[2];

    // load policy file
    if(!_mdp->readDualOptimal(gbm, &string(filename)[0], &list)) {
        throw cpputil::InvalidException("SPUDD's readDualOptimal() failed.");
    }
    _act = list[0];
    Cudd_Ref(_act);
    _val = list[1];
    Cudd_Ref(_val);
    LOG_INFO("Read spudd policy file.");

    delete [] list;
}

Action _SpuddPolicy::getAction(const State& s) {
    FactorVec svec = resolve(_domain, s);
    vector<int> state(svec.begin(),svec.end());
    int optact = _mdp->consultOptimalPolicy(_act, _val, state.data());
    LOG_DEBUG("For state " << s << " return action: " << _mdp->getActionName(optact));

    return Action(_domain, optact);
};

} // namespace crl
