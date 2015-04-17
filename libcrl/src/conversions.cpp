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

//
// Helper functions
//

/// \brief Return the array representation of a \a State
FactorVec resolve(const Domain& dom, const State& s) {
  FactorVec fvec;
  fvec.reserve(dom->getNumStateFactors());
  for(Size i = 0; i < dom->getNumStateFactors(); i++) {
      fvec.push_back(s.getFactor(i));
  }
  return fvec;
}
#if 0
/// \brief Return the array representation of an \a Action
FactorVec resolve(const Domain& dom, const Action& a) {
  FactorVec fvec;
  fvec.reserve(dom->getNumActionFactors());
  for(Size i = 0; i < dom->getNumActionFactors(); i++) {
      fvec.push_back(a.getFactor(i));
  }
  return fvec;
}
#endif
/// \brief Count the number of different factors between a, b (from the same domain)
/// \return The number of different elements and a bit-mask (0,1) where a and b differ
tuple<Size,vector<bool>> count_unique(const Domain& dom, const FactorVec& a, const FactorVec& b) {
  assert(a.size() == b.size());
  Size count = 0;
  vector<bool> mask(a.size(),0);
  for(Size i = 0; i < a.size(); i++) {
      if(a[i] != b[i]) {
          count++;
          mask[i] = 1;
      }
  }
  return make_tuple(count,mask);
}

} // anonymous ns

namespace crl {

/// \brief This prints out a specific function (i.e., DBNFactor or LRF) in SPUDD format
/// \param fp The file to write to
/// \param sf The specific function (e.g., either DBNFactor or LRF)
/// \param a The relevant (local) part of the considered joint action
/// \param printer A custom printer function for this function
template<class T, class F>
void writeFunction(ofstream& fp, const _DiscreteFunction<T>* sf, const Action& a, F printer) {
  Domain subdomain = sf->getSubdomain();

  // loop over instantiations of state variables inside scope of this factor
  _StateIncrementIterator sitr(subdomain);
  const Size sdom_no = subdomain->getNumStateFactors();
  FactorVec pvec(sdom_no, 1); // vector representation of previous state
  bool firstIter = true;

  do { // for each permutation
      const State& s = sitr.next();
      const FactorVec svec = resolve(subdomain, s);

      // close previously opened variable blocks
      auto count = count_unique(subdomain, svec, pvec);
      if(!firstIter) {
        fp << string(2*std::get<0>(count),')') << endl;
      }
      else {
        firstIter = false;
      }

      // check where indices changed from previous iteration
      const vector<bool>& mask = std::get<1>(count);
      const RangeVec& r = subdomain->getStateRanges();
      const StrVec& s_names = subdomain->getStateNames();
      for(int i = 0; i < mask.size(); i++) {
          if(mask[i]) { // there was a change at `i'
              if(svec[i] == r[i].getMin()) { // a transition into the first factor value
                  fp << " (" << s_names[i];
              }
              fp << " (" << svec[i]; // print value
          }
      }

      // call custom print for this function
      printer(s);

      pvec = svec;

  } while(sitr.hasNext());
  // write out last closing braces
  fp << string(sdom_no*2+1,')') << endl << endl;
}

int exportToSpudd(FactoredMDP fmdp, Domain domain, float gamma, const string& problemName, const string& filename)
{
  // input checking
  if(fmdp == nullptr || filename.empty()) {
      cerr << "libcrl::exportToSpudd: invalid parameters" << endl;
      return 1;
  }
  ofstream fp(filename.c_str());
  if(!fp.is_open()) {
      cerr << "libcrl::exportToSpudd: failed to open file " << filename << endl;
      return 2;
  }
  const _DBN& dbn = fmdp->T();
  if(dbn.hasConcurrentDependency()) {
      cerr << "libcrl::exportToSpudd: spudd does not currently support concurrent dependencies in transition function" << endl;
      return 3;
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

  // write actions (unfactored)
  const StrVec& a_str_vec = domain->getActionNames();
  _ActionIncrementIterator jaitr(domain);
  FactorIterator fitr = dbn.factors();

  while (jaitr.hasNext()) {
      // construct and print joint action name
      const Action& ja = jaitr.next();
      fp << "action " << concat(ja, a_str_vec) << endl;

      // write out CPT for each state factor
      Size fidx = 0;
      fitr->reset();
      while(fitr->hasNext()) {
          fp << s_str_vec[fidx++] << endl;
          // determine relevant action subset for current state factor
          const DBNFactor& sf = fitr->next();
          const Action& a = sf->mapAction(ja);

          // print out a probability distribution
          writeFunction(fp, sf.get(), a, [&](const State& ls) {
              // write distribution as vector
              const ProbabilityVec& dist = sf->T(ls, a);
              fp << " (";
              for(auto pr : dist) {
                  fp << pr << " ";
              }
          });
      }

      // write generic cost term
      fp << "cost [+" << endl;

      for(const auto& lrf : fmdp->getLRFs()) {
          // determine relevant action subset for current lrf
          const Action& a = lrf->mapAction(ja);

          // print out a reward function
          writeFunction(fp, lrf.get(), a, [&](const State& ls) {
              // write reward as cost for this instantiation
              double reward = (*lrf)(ls,a);
              fp << " (" << -reward;
          });
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

    return 0;
}

string concat(const RLType& jt, const StrVec& names) {
  stringstream ss;
  for (Size i=0; i<jt.size(); i++) {
      ss << names[i] << "_" << jt.getFactor(i) << "__";
  }
  string tname = ss.str();
  return tname.substr(0, tname.length()-2);
}

}
