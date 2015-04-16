/*
    Copyright 2015 Philipp Robbel

    License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.

    The software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
 */

#include <gtest/gtest.h>

#include "crl/alp.hpp"
#include "crl/alp_lpsolve.hpp"
#include "crl/env_sysadmin.hpp"
#include "crl/vi.hpp"
#include "logger.hpp"

using namespace std;
using namespace crl;
using namespace cpputil;

//
// More complex integration test that runs the Backprojections on the SysAdmin DBN
//

namespace {

} // anonymous namespace

TEST(DBNIntegrationTest, TestBackproject) {
    srand(time(NULL));

    sysadmin::Sysadmin thesys = buildSysadmin("star", 3);
    Domain domain = thesys->getDomain();

    FactoredMDP fmdp = thesys->getFactoredMDP();
    const _DBN& T2 = fmdp->T();
    LOG_INFO(T2);

    // Define two random basis functions over some subset of state factors
    FDiscreteFunction<double> F1 = boost::make_shared<_FDiscreteFunction<double>>(domain);
    F1->addStateFactor(0);
    F1->addStateFactor(3);
    F1->pack(); // allocate flat table

    FDiscreteFunction<double> F2 = boost::make_shared<_FDiscreteFunction<double>>(domain);
    F2->addStateFactor(2);
    F2->addStateFactor(3);
    F2->addStateFactor(5);
    F2->pack(); // allocate flat table

    // randomize function values
    auto& vals  = F1->values();
    std::generate(vals.begin(), vals.end(), [&]{ return cpputil::randDouble(); });
    auto& vals2 = F2->values();
    std::generate(vals2.begin(), vals2.end(), [&]{ return cpputil::randDouble(); });

    // Backproject those functions through the DBN
    _Backprojection<double> B1(domain, T2, F1, "bp1");
    B1.cache(); // compute them exhaustively
    _Backprojection<double> B2(domain, T2, F2, "bp2");
    B2.cache(); // compute them exhaustively

    //
    // Compare expectation over successor states in domain to sum over backprojections
    //
    _StateActionIncrementIterator saitr(domain);
    while(saitr.hasNext()) {
        const std::tuple<State,Action>& sa = saitr.next();
        const State& s0  = std::get<0>(sa);
        const Action& a0 = std::get<1>(sa);
        double sum = 0.;
        // over all successor states
        _StateIncrementIterator sitr(domain);
        while(sitr.hasNext()) {
            const State& s = sitr.next();
            double p_dbn = T2.T(s0, a0, s);
            State ms = F1->mapState(s);
            const double v1 = (*F1)(ms,Action());
            ms = F2->mapState(s);
            const double v2 = (*F2)(ms,Action());
            sum += p_dbn * (v1+v2);
        }

        // compare with sum over backprojections
        double bsum = 0.;
        State ms  = B1.mapState(s0);
        Action ma = B1.mapAction(a0);
        bsum += B1(ms,ma);
        ms = B2.mapState(s0);
        ma = B2.mapAction(a0);
        bsum += B2(ms,ma);
        EXPECT_DOUBLE_EQ(sum, bsum);
    }
}
