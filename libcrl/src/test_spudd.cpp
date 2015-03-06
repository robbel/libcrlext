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
    Domain domain;

    exportToSpudd(fmdp, domain, 0.99, "bla", "blub");

//    crl::exportToSpudd(FactoredMDP mdp,
//                       float gamma,
//                       const std::string& problemName,
//                       const std::string& filename
//                       );

}


#if 0
void FactoredDecPOMDPDiscrete::ExportSpuddFile(const string& filename) const
{
  // write header
  ...

  // write variables
  ...

  // write actions
  const Scope& jAsc = GetAllAgentScope();
  for(Index jaI = 0; jaI < GetNrJointActions(); jaI++) {
    vector<Index> A = JointToIndividualActionIndices(jaI);

    // construct and print joint action name
    ...

    // write out CPT for each state factor
    for(Index y = 0; y < GetNrStateFactors(); y++) {
      fp << GetStateFactorDiscrete(y)->GetName() << endl;

      // figure out action subset for ii
      const Scope& ASoI_y = Get2DBN()->GetASoI_Y(y);
      size_t ASoI_y_size = ASoI_y.size();
      vector<Index> As_restr(ASoI_y_size);      // this is the action subset
      IndexTools::RestrictIndividualIndicesToNarrowerScope(A, jAsc, ASoI_y, As_restr);

      // loop over X instantiations
      const Scope& XSoI_y  = Get2DBN()->GetXSoI_Y(y);
      size_t XSoI_y_size = XSoI_y.size(); // number of variables X in y's scope
      vector<size_t> r_nrX = Get2DBN()->GetNrVals_XSoI_Y(y); // number of values for X
      vector<Index> Xs(XSoI_y_size, 0 ); // instantiation for X variables in XSoI_y
      vector<Index> prevXs(XSoI_y_size, 1 ); // previous iteration
      const Scope emptySc; // for Y variables (no within-stage influences now)
      bool firstIter = true;

      do { // for each permutation
        //cout << SoftPrintVector(Xs) << endl;
        // close previously opened variable blocks
        size_t nrXchanges = 0;
        for(Index scI=0; scI < XSoI_y_size; scI++) { // for all state factors in ii
          if(prevXs[scI] != Xs[scI])
            nrXchanges++;
        }
        if(!firstIter) fp << string(2*nrXchanges,')') << endl; else firstIter = false;

        // check where indices changed from previous iteration
        for(Index scI=0; scI < XSoI_y_size; scI++) { // for all state factors in ii
          Index sfI = XSoI_y.at(scI);
          const StateFactorDiscrete* sfac = GetStateFactorDiscrete(sfI);

          if(prevXs[scI] != Xs[scI]) {
            if(Xs[scI] == 0) {
              // write out next variable name
              string name = sfac->GetName();
              fp << " (" << name;
            }
            // print variable value
            string value = sfac->GetStateFactorValue(Xs[scI]);
            fp << " (" << value;
          }
        }

        // write distribution as vector
        vector<double> dist = Get2DBN()->GetYProbabilitiesExactScopes(Xs, As_restr, emptySc, y);
        // if(p > Globals::PROB_PRECISION)
        fp << " (";
        for(vector<double>::const_iterator pI = dist.begin(); pI != dist.end(); ++pI)
          fp << *pI << " ";

        prevXs = Xs;

      } while(! IndexTools::Increment( Xs, r_nrX ) );
      // write out last closing braces
      fp << string(XSoI_y_size*2+1,')') << endl << endl;
    }

    // write generic cost term
    fp << "cost [+" << endl;
    for(Index rI=0; rI < GetNrLRFs(); rI++) {
      const Scope& agSC = GetAgentScopeForLRF(rI);
      size_t agSC_size = agSC.size();
      vector<Index> As_restr(agSC_size);
      IndexTools::RestrictIndividualIndicesToNarrowerScope(A, jAsc, agSC, As_restr);

      //XXX this lookup can be replaced with cached values
      // in _m_nrSFVals.at(LRF), cf. line 286
      const vector< size_t>& nrVals = GetNrValuesPerFactor(); //XXX move out here
      const Scope& sfSC = GetStateFactorScopeForLRF(rI);
      size_t sfSC_size = sfSC.size();
      vector< size_t> restrXVals(sfSC_size);
      IndexTools::RestrictIndividualIndicesToScope(nrVals, sfSC, restrXVals);

      vector<Index> Xs2(sfSC_size, 0 );
      vector<Index> prevXs2(sfSC_size, 1 ); // previous iteration
      const Scope emptySc;
      bool firstIter = true;

      do { // for each permutation
        // close previously opened variable blocks
        size_t nrXchanges = 0;
        for(Index scI=0; scI < sfSC_size; scI++) { // for all ii state factors
          if(prevXs2[scI] != Xs2[scI])
            nrXchanges++;
        }
        if(!firstIter) fp << string(2*nrXchanges,')') << endl; else firstIter = false;

        // check where indices changed from previous iteration
        for(Index scI=0; scI < sfSC_size; scI++) { // for all ii state factors
          Index sfI = sfSC.at(scI);
          const StateFactorDiscrete* sfac = GetStateFactorDiscrete(sfI);

          if(prevXs2[scI] != Xs2[scI]) {
            if(Xs2[scI] == 0) {
              // write out next variable name
              string name = sfac->GetName();
              fp << " (" << name;
            }
            // print variable value
            string value = sfac->GetStateFactorValue(Xs2[scI]);
            fp << " (" << value;
          }
        }

        // write reward as cost for this ii instantiation
        double reward = GetLRFReward(rI, Xs2, As_restr);
        fp << " (" << -reward;

        prevXs2 = Xs2;

      } while(! IndexTools::Increment( Xs2, restrXVals ) );
      // write out last closing braces
      fp << string(sfSC_size*2+1,')') << endl;
    }
    fp << "     ]" << endl
       << "endaction" << endl << endl;
  }

  // write reward function (note: subsumed as costs inside each individual action)
  fp << "reward (0.0)" << endl << endl;

  // write footer
  fp << "discount " << GetDiscount() << endl //XXX add warning
     << "//horizon 10" << endl
     << "tolerance 0.1" << endl;
}
#endif
