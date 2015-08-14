/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include <gtest/gtest.h>
#include <fstream>

#include "crl/alp.hpp"
#include "crl/basis_gen.hpp"
#include "crl/env_graphprop.hpp"
#include "crl/vi.hpp"
#include "crl/conversions.hpp"
#include "logger.hpp"

using namespace std;
using namespace crl;
using namespace cpputil;

//
// More complex integration test that runs the ALP solver and basis discovery for the GraphProp environment
//

TEST(ALPIntegrationTest, TestGraphpropBasisDiscovery) {
  srand(time(NULL));

  string cfg = "../../graphprop/cfgs/default.xml.100-5";
  string data = "../../graphprop/data/Gnp100_msp.txt";
  ifstream iscfg(cfg);
  ifstream isdat(data);
  graphprop::GraphProp thegrp_lift, thegrp;
  // read `lifted' version of mdp
  if(!(thegrp_lift = readGraphProp(iscfg, isdat, true))) {
    LOG_ERROR("Error while reading from " << cfg << " or " << data);
    FAIL();
  }
  // read flat version of same mdp
  iscfg.clear(); // clear fail and eof bits
  isdat.clear();
  iscfg.seekg(0, std::ios::beg); // beginning of file
  isdat.seekg(0, std::ios::beg); // beginning of file
  if(!(thegrp = readGraphProp(iscfg, isdat, false))) {
    LOG_ERROR("Error while reading from " << cfg << " or " << data);
    FAIL();
  }

  Domain domain_lift = thegrp_lift->getDomain();
  FactoredMDP fmdp_lift = thegrp_lift->getFactoredMDP();
  FactoredValueFunction fval_lift = boost::make_shared<_FactoredValueFunction>(domain_lift);
  LOG_INFO(fmdp_lift->T());
  Domain domain = thegrp->getDomain();
  FactoredMDP fmdp = thegrp->getFactoredMDP();
  FactoredValueFunction fval = boost::make_shared<_FactoredValueFunction>(domain);

  // add the constant basis (to guarantee LP feasibility)
  //auto cfn = boost::make_shared<_ConstantFn<Reward>>(domain);
  //fval_lift->addBasisFunction(std::move(cfn), 0.);
  // add singleton basis functions
  const RangeVec& ranges_lift = domain_lift->getStateRanges();
  for(Size fa = 0; fa < ranges_lift.size(); fa++) { // assumption: DBN covers all domain variables
      auto I_o = boost::make_shared<_Indicator<Reward>>(domain_lift, SizeVec({fa}), State(domain_lift,0));
      _StateIncrementIterator sitr(I_o->getSubdomain());
      while(sitr.hasNext()) {
          auto I = boost::make_shared<_Indicator<Reward>>(domain_lift, SizeVec({fa}), sitr.next());
          fval_lift->addBasisFunction(std::move(I), 0.);
      }
  }
  for(Size fa = 0; fa < ranges_lift.size(); fa++) {
      auto I_o = boost::make_shared<_Indicator<Reward>>(domain, SizeVec({fa}), State(domain,0));
      _StateIncrementIterator sitr(I_o->getSubdomain());
      while(sitr.hasNext()) {
          auto I = boost::make_shared<_Indicator<Reward>>(domain, SizeVec({fa}), sitr.next());
          fval->addBasisFunction(std::move(I), 0.);
      }
  }

  // obtain initial solution
  _ALPPlanner planner(fmdp_lift, 0.9);
  _ALPPlanner basisplan(fmdp, 0.9);

  planner.setFactoredValueFunction(fval_lift);
  basisplan.setFactoredValueFunction(fval);
  long start_time = time_in_milli();
  int res = planner.plan();
  long end_time = time_in_milli();
  LOG_INFO("FALP planner returned after " << end_time - start_time << "ms");

  EXPECT_EQ(res, 0) << "ALP " << (res == 1 ? "generateLP()" : "solve()") << " failed"; // else: lp successfully generated

  if(!res) { // success
    LOG_INFO("Results:");
    for(auto w : fval_lift->getWeight()) {
        LOG_INFO(" W: " << w);
    }

    LOG_INFO("Writing initial basis function weights to weights0.txt");
    {
      ofstream file("weights0.txt");
      for(double w : fval_lift->getWeight()) {
        file << std::fixed << std::setprecision(std::numeric_limits<double>::digits10+2) << w << '\n';
      }
    }

    // transfer solution into second mdp
    vector<double>& lift_weights = fval_lift->getWeight(); // solution
    vector<double>& weights = fval->getWeight(); // target
    std::copy(lift_weights.begin(), lift_weights.end(), weights.begin());
    basisplan.precompute();

    // Compute initial Bellman error
    const SizeVec elim_order_s = cpputil::ordered_vec<Size>(domain->getNumStateFactors());
    double beval = algorithm::factoredBellmanError(domain, fval, elim_order_s);
    LOG_INFO("Bellman error (0): " << beval);

    //
    // Basis generation
    //
    BinaryBasisGenerator<NChooseTwoIterator<Size,SizeVec>,OptBEBFScore> basisGen(domain, fval, fmdp, "optbebf-test");
    for(int k = 0; k < 20; k++) {
        DiscreteFunction<Reward> nextBasis = basisGen.nextBest();
        if(!nextBasis) {
          LOG_INFO("No next best basis: cost deemed infinite by Scoring function.");
          break;
        }

        // avoid double-insertion of same feature
        const Conjunction* cf = dynamic_cast<const Conjunction*>(nextBasis.get());
        assert(cf);
        basisGen.addToBlacklist(*cf);
        // insert feature into basis
        fval_lift->clearBackprojections(); // TODO: implement SWAP so that recomputation of previous ones avoided
        fval->clearBackprojections();
        fval_lift->addBasisFunction(nextBasis, 0.);
        fval->addBasisFunction(nextBasis, 0.);

        // run iteration of planning
        long start_time = time_in_milli();
        // TODO: can init with old weights + 0 for valid solution..
        // TODO: don't have to deallocate lp every iteration
        int res2 = planner.plan();
        long end_time = time_in_milli();
        LOG_INFO("2nd: FALP planner returned after " << end_time - start_time << "ms");

        EXPECT_EQ(res2, 0) << "ALP " << (res2 == 1 ? "generateLP()" : "solve()") << " failed"; // else: lp successfully generated

        if(!res2) { // success
            LOG_INFO("2nd: Results:");
            for(auto w : fval_lift->getWeight()) {
                LOG_INFO(" W: " << w);
            }

            // transfer solution into second mdp
            vector<double>& lift_weights = fval_lift->getWeight(); // solution
            vector<double>& weights = fval->getWeight(); // target
            std::copy(lift_weights.begin(), lift_weights.end(), weights.begin());
            basisplan.precompute();

            LOG_INFO("Writing basis function weights to text file");
            {
                ofstream file("weights" + to_string(k+1) + ".txt");
                for(double w : fval_lift->getWeight()) {
                    file << std::fixed << std::setprecision(std::numeric_limits<double>::digits10+2) << w << '\n';
                }
            }
            LOG_INFO("Writing conjunctive basis to text file");
            {
                ofstream file("conj" + to_string(k+1) + ".txt");
                for(const auto& fn : fval_lift->getBasis()) {
                    const Conjunction* pc = dynamic_cast<const Conjunction*>(fn.get());
                    if(pc) {
                        const auto& baseFeatures = pc->getBaseFeatures();
                        for(int i = 0; i < baseFeatures.size(); i++) {
                            file << baseFeatures[i] << (i < baseFeatures.size()-1 ? "," : "");
                        }
                        file << '\n';
                    }
                }
            }

            // Compute Bellman error
            double beval = algorithm::factoredBellmanError(domain, fval, elim_order_s);
            LOG_INFO("Bellman error (" << k << "): " << beval);
        }
    }

  }
}
