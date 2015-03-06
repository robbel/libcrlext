/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include <iostream>
#include <string>
#include <fstream>
#include "cpputil.hpp"
#include "crl/conversions.hpp"

using namespace std;
using namespace crl;

namespace {

/// \brief Convert supplied (joint) action into a string, given the action_names from the \a Domain.
string concat(const Action& ja, const StrVec& action_names) {
  stringstream ss;
  for (Size i=0; i<ja.size(); i++) {
      ss << action_names[i] << "_" << ja.getFactor(i) << "__";
  }
  string aname = ss.str();
  return aname.substr(0, aname.length()-2);
}

} // anonymous namespace


namespace crl {

void exportToSpudd(FactoredMDP fmdp, Domain domain, float gamma, const string& problemName, const string& filename)
{
  // input checking
  if(fmdp == nullptr || filename.empty()) {
      cerr << "libcrl::exportToSpudd: invalid parameters" << endl;
      return;
  }
  ofstream fp(filename.c_str());
  if(!fp.is_open()) {
      cerr << "libcrl::exportToSpudd: failed to open file " << filename << endl;
      return;
  }
  const DBN dbn = fmdp->T();
  if(dbn->hasConcurrentDependency()) {
      cerr << "libcrl::exportToSpudd: spudd does not currently support concurrent dependencies in transition function" << endl;
      return;
  }

  // write spudd format header
  fp << "// Automatically produced by libcrl::exportToSpudd"
     << endl << "// SPUDD / Symbolic Perseus Format for '" << problemName << "'"
     << endl << endl;

  // write variables
  const RangeVec& s_range_vec = domain->getStateRanges();
  const StrVec& s_str_vec = domain->getStateNames();
  fp << "(variables" << endl;
  for(Size i = 0; i < domain->getNumStateFactors(); i++) {
      fp << " (" << s_str_vec[i];
      const FactorRange& v = s_range_vec[i];
      for(Factor f = v.getMin(); f <= v.getMax(); f++) { // Note: it's a closed interval
        fp << " " << f;
      }
      fp << ")" << endl;
  }
  fp << ")" << endl << endl;

  // write actions
  const StrVec& a_str_vec = domain->getActionNames();
  _ActionIncrementIterator jaitr(domain);
  FactorIterator fitr = dbn->factors();

  while (jaitr.hasNext()) {
      // construct and print joint action name
      Action ja = jaitr.next();
      fp << "action " << concat(ja, a_str_vec) << endl;

      // write out CPT for each state factor
      Size fidx = 0;
      fitr->reset();
      while(fitr->hasNext()) {
          fp << s_str_vec[fidx++] << endl;

          // figure out action subset for current state factor
          DBNFactor sf = fitr->next();
          Domain subdomain = sf->getSubdomain();
          const Action a = sf->mapAction(ja);

          // loop over instantiations of state variables inside scope of this factor
          _StateIncrementIterator sitr(subdomain);
          //bool firstIter = true;

          do { // for each permutation
              State s = sitr.next();

              // close previously opened variable blocks
              // TODO

              // check where indices changed from previous iteration
              // TODO

              // write distribution as vector
              const ProbabilityVec& dist = sf->T(s, a);
              fp << " (";
              for(auto pr : dist) {
                  fp << pr << " ";
              }

              // TODO

          } while(sitr.hasNext());
          // write out last closing braces
          fp << string(subdomain->getNumStateFactors()*2+1,')') << endl << endl;
      }

      // write generic cost term
      fp << "cost [+" << endl;
      _StateIncrementIterator jsitr(domain);
      while(jsitr.hasNext()) {
          State js = jsitr.next();

          // close previously opened variable blocks
          // TODO

          double reward = fmdp->R(js,ja);
          fp << " (" << -reward;
      }
      fp << "     ]" << endl
         << "endaction" << endl << endl;
    }

    // write reward function (note: subsumed as costs inside each individual action)
    fp << "reward (0.0)" << endl << endl;

    // write footer
    fp << "discount " << gamma << endl //XXX add warning
       << "//horizon 10" << endl
       << "tolerance 0.1" << endl;
}

}
