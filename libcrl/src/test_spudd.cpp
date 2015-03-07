/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include <iostream>
#include "crl/conversions.hpp"
#include "crl/factor_learner.hpp"
#include "cpputil.hpp"

using namespace std;
using namespace crl;

FactoredMDP makeFactoredMDP(Domain domain) {

  FactoredMDP fmdp = boost::make_shared<_FactoredMDP>(domain);

  const RangeVec& ranges = domain->getStateRanges();
  for(Size y = 0; y < domain->getNumStateFactors(); y++) {
      // create a dbn factor
      DBNFactor fa = boost::make_shared<_DBNFactor>(domain, y);
      fa->addDelayedDependency(y); // dependence on self
      fa->addActionDependency(0); // dependence on first action factor
      fa->pack();
      // fill with random values for transition fn

//    time_t start_time = time_in_milli();
      Domain subdomain = fa->getSubdomain();
      for (Size state_index=0; state_index<subdomain->getNumStates(); state_index++) {
              State s(subdomain, state_index);
              for (Size action_index=0; action_index<subdomain->getNumActions(); action_index++) {
                      Action a(subdomain, action_index);
                      //mdp->setR(s, a, randDouble()); // FIXME: empty rewards, for now
                      Probability total = 0.;
                      ProbabilityVec probs(ranges[y].getSpan()+1);
                      for(Factor f=0; f<=ranges[y].getSpan(); f++) { // we don't care about actual factor value here
                          Probability prob = cpputil::randDouble();
                          total += prob;
                          probs[f] = prob;
                      }
                      // normalize
                      for(Factor f=0; f<=ranges[y].getSpan(); f++) {
                          Probability prob = probs[f]/total;
                          fa->setT(s,a,ranges[y].getMin()+f,prob); // here we care about actual factor value
                      }
              }
      }
//    time_t end_time = time_in_milli();
//    cout << "created DBNFactor in " << end_time - start_time << "ms" << endl;

      // add random factor to dbn
      fmdp->addDBNFactor(fa);
  }

  return fmdp;
}

MDP convertToMDP(FactoredMDP fmdp) {

  const Domain& domain = fmdp->getDomain();
  _FMDP mdp(domain);

  for (Size state_index=0; state_index<domain->getNumStates(); state_index++) {
          State s(domain, state_index);
          for (Size action_index=0; action_index<domain->getNumActions(); action_index++) {
                  Action a(domain, action_index);
                  mdp.setR(s, a, fmdp->R(s, a));
                  for (Size next_index=0; next_index<domain->getNumStates(); next_index++) {
                          State n(domain, next_index);
                          Probability p = fmdp->T(s,a,n);
                          mdp.setT(s, a, n, p);
                  }
          }
  }

  return boost::make_shared<_FMDP>(mdp);
}

int main()
{
  try {
    srand(0);

    Domain domain = boost::make_shared<_Domain>();
    domain->addStateFactor(0, 299, "first_state"); // 300 states
    domain->addActionFactor(0, 4, "first_agent");  // 5 actions
    domain->setRewardRange(-1, 0);

    FactoredMDP fmdp = makeFactoredMDP(domain);

    exportToSpudd(fmdp, domain, 0.99, "test", "test.spudd");

    MDP mdp = convertToMDP(fmdp);
  }
  catch(const cpputil::Exception& e) {
    cerr << e << endl;
  }
}
