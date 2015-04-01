#include <gtest/gtest.h>

#include "crl/alp.hpp"

using namespace std;
using namespace crl;

///
/// \brief This is just some nonsensical placeholder for now!
///
TEST(DBNTest, Noop) {
  srand(time(NULL));

  Domain domain = boost::make_shared<_Domain>();
  domain->addStateFactor(0, 299, "first_state"); // 300 states
  domain->addActionFactor(0, 4, "first_agent");  // 5 actions
  domain->setRewardRange(-1, 0);

  State s(domain, 1);
  State s2(domain,2);
  Action a(domain,0);

  Indicator I = boost::make_shared<_Indicator>(domain, domain, s);
  assert((*I)(s) && !(*I)(s2));

  DBN dbn;
  dbn->T(s,a,s,identity_map,identity_map,identity_map);

  _Backprojection<double> b(domain, dbn, I);

//  Backprojection<int> b = boost::make_shared<_Backprojection<int>>(dbn,I);

  EXPECT_EQ(1,1);
}
