/*
    Copyright 2008 Rutgers University
    Copyright 2008 John Asmuth

    This file is part of CRL.

    CRL is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    CRL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with CRL.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#include <iostream>
#include "crl/hdomain.hpp"

using namespace std;
using namespace crl;



_HQTable::_HQTable(const Domain& domain, Size num_buckets)
: _HStateActionTable<Reward>(domain), _best_actions(alloc_hash_map<Size,Action,SizeHash>(num_buckets)),
  _best_qs(alloc_hash_map<Size,Reward,SizeHash>(num_buckets)) {

}

void _HQTable::setQ(const State& s, const Action& a, Reward r) {
	Size index = s.getIndex();
	_HStateActionTable<Reward>::setValue(s, a, r);
	if (!_best_actions[index]) {
		_best_actions[index] = Action(_domain, 0);
	}
	if (_best_actions[index] == a) {
		_best_qs[index] = r;
		_ActionIncrementIterator itr(_domain);
		while (itr.hasNext()) {
			Action fa = itr.next();
			Reward q = getQ(s, fa);
			if (q > _best_qs[index]) {
				_best_qs[index] = q;
				_best_actions[index] = fa;
			} 
		}
	}
	if (r > _best_qs[index]) {
		_best_qs[index] = r;
		_best_actions[index] = a;
	}
}

