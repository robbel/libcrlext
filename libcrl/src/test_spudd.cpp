/*
    Copyright 2015 Philipp Robbel

    TODO: ADD LICENSE
 */

#include <iostream>
#include "crl/conversions.hpp"

using namespace std;
using namespace crl;

int main()
{
    cout<<"hello"<<endl;

    FactoredMDP fmdp;

    exportToSpudd(fmdp,0.99,"bla","blub");

//    crl::exportToSpudd(FactoredMDP mdp,
//                       float gamma,
//                       const std::string& problemName,
//                       const std::string& filename
//                       );

}
