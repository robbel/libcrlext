/*
    Copyright 2015 Philipp Robbel

    TODO: ADD LICENSE
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
string toString(const Action& act, const StrVec& action_names) {
  stringstream ss;
  for (Size i=0; i<act.size(); i++) {
      ss << action_names[i] << "_" << act.getFactor(i) << "__";
  }
  string aname = ss.str();
  return aname.substr(0, aname.length()-2);
}

} // anonymous namespace

namespace crl {

// STILL NEED:
// - (factored) reward function..
// - name of problem, name of factors (state, action)
void exportToSpudd(FactoredMDP fmdp, Domain domain, float gamma, const string& problemName, const string& filename)
{
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

  // write header
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
  // Alternative:
  //  for (Size action_index=0; action_index<domain->getNumActions(); action_index++)
  //    Action a(domain, action_index);

  while (jaitr.hasNext()) {
#if 0
      vector<Index> A = JointToIndividualActionIndices(jaI);
#endif
      Action ja = jaitr.next(); // joint action
      // construct and print joint action name
      fp << "action " << toString(ja, a_str_vec) << endl;

      // write out CPT for each state factor
      Size fidx = 0;
      FactorIterator fitr = dbn->factors();
      // Alternative:
      //  for(Size y = 0; y < domain->getNumStateFactors(); y++) { }
      while(fitr->hasNext()) {
          DBNFactor sf = fitr->next();
          Domain subdomain = sf->getSubdomain();
          const Action a = sf->mapAction(ja); // map joint action to factor-relevant subset
          fp << s_str_vec[fidx++] << endl;

          // loop over X instantiations
          _StateIncrementIterator sitr(subdomain);
          do { // for each permutation
              // close previously opened variable blocks



              State s = sitr.next(); // the state factor subset in subdomain





              const ProbabilityVec& dist = sf->T(s, a);
              fp << " (";
              for(auto pr : dist) {
                  fp << pr << " ";
              }




          } while(sitr.hasNext());
          // write out last closing braces


#if 0
        // figure out action subset for ii
        const Scope& ASoI_y = Get2DBN()->GetASoI_Y(y);      // action scope of that factor y
        size_t ASoI_y_size = ASoI_y.size();                 // # actions
        vector<Index> As_restr(ASoI_y_size);                // restricted action set
        IndexTools::RestrictIndividualIndicesToNarrowerScope(A, jAsc, ASoI_y, As_restr);

        // loop over X instantiations
        const Scope& XSoI_y  = Get2DBN()->GetXSoI_Y(y);
        size_t XSoI_y_size = XSoI_y.size(); // number of variables X in y's scope
        vector<size_t> r_nrX = Get2DBN()->GetNrVals_XSoI_Y(y); // number of values for X
        vector<Index> Xs(XSoI_y_size, 0 ); // instantiation for X variables in XSoI_y
        vector<Index> prevXs(XSoI_y_size, 1 ); // previous iteration
        const Scope emptySc; // for Y variables (no within-stage influences now)
        bool firstIter = true;

        do { // for each permutation in Xs
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
#endif
      }
#if 0
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
#endif
    }

    // write reward function (note: subsumed as costs inside each individual action)
    fp << "reward (0.0)" << endl << endl;

    // write footer
    fp << "discount " << gamma << endl //XXX add warning
       << "//horizon 10" << endl
       << "tolerance 0.1" << endl;
}

}
