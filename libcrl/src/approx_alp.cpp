/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include "crl/approx_alp.hpp"
#include "logger.hpp"

using namespace std;
using namespace dai;

namespace crl {

#if defined(DAI_WITH_BP) && defined(DAI_WITH_JTREE)

void _ApproxALP::buildFactorGraph() {

  // TODO

}

namespace testing {

int maxplus_demo() {
  // Ternary, binary, and ternary variable
  Var x0(0,3), x1(1,2), x2(2,3);

  VarSet vs1 (x0);
  VarSet vs2 (x0, x1);
  VarSet vs3 (x1, x2);
  cout << "Size of statespace of vs3 = " << vs3.nrStates() << endl;

  VarSet vs_all;
  vs_all = vs3|x0;
  cout << "Size of statespace of vsall = " << vs_all.nrStates() << endl;
  cout << "Size of statespace of vs3   = " << vs3.nrStates() << endl;

  // Define factors
  double values0[3][3] = {{0,6,0}, {0,0,0}, {2,2,2}};         //[x2][x0]  e.g., for 6 -> x0=1 x2=0
  double values1[2][3] = {{0.0, 0.0, 1.0} ,{1.0, 0.0, 1.0}};  //[x1][x0]
  double values2[3][2] = {{0.0, 0.0}, {1.0,1.0}, {0.0, 1.0}}; //[x2][x1]

  double * ptr_to_values0 = &values0[0][0];
  double * ptr_to_values1 = &values1[0][0];
  double * ptr_to_values2 = &values2[0][0];

  dai::Factor f0( VarSet(x0, x2), ptr_to_values0 );
  dai::Factor f1( VarSet(x0, x1), ptr_to_values1 );
  dai::Factor f2( VarSet(x1, x2), ptr_to_values2 );

  // Define FactorGraph
  vector<dai::Factor> facs;
  facs.push_back(f0);
  facs.push_back(f1);
  facs.push_back(f2);
  FactorGraph fg(facs);
  cout << "FactorGraph: "<< endl << fg << endl;

  // Max-plus properties
  size_t  maxiter = 10000;
  double  tol = 1e-9;
  size_t  verb = 4;

  // Store the constants in a PropertySet object
  PropertySet opts;
  opts.set("maxiter",maxiter);
  opts.set("tol",tol);
  opts.set("verbose",verb);
  opts.set("updates",string("SEQRND")); // PARALL, SEQFIX, ..

  // Run the algorithm
  BP mp(fg, opts("logdomain",false)("inference",string("MAXPLUS"))("damping",string("0.0")));
  mp.init();
  mp.run();

  cout << "Approximate (max-plus) single node marginals:" << endl;
  for( size_t i = 0; i < fg.nrVars(); i++ )
      cout << mp.belief(fg.var(i)) << endl;

  // Calculate joint state of all variables that has maximum value
  vector<size_t> mpstate = mp.findMaximum();

  cout << "Approximate (max-plus) MAP state: " << endl;
  for( size_t i = 0; i < mpstate.size(); i++ )
      cout << fg.var(i) << ": " << mpstate[i] << endl;

  return 0;
}

} // namespace testing

#endif

} // namespace crl
