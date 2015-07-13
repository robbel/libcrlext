function buildFG(nv,nrA,edges,outlier)

%buildFG(nv,nrA,numofN) - build a random coordination graph
%
%  nv      - # of nodes
%  nrA     - # of actions per agent
%  edges   - # of edges in the graph

% Copyright 2015 Philipp Robbel
% Copyright 2004 Nikos Vlassis
%
% License: http://www.gnu.org/licenses/lgpl LGPL version 3.0, or (at your option) any later version.
% 
% The software is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.

global CG;

CG.node   = cell(nv,1);                   % CG contains of nv agents
CG.numofA = nrA;                          % store number of actions

for i = 1:nv                            % for every agent
  CG.node{i}.id    = i;                 %  set its id
  CG.node{i}.non   = 0;                 %  0 neighbors 
  CG.node{i}.neigh = [];                %  no neighbor references
end

numofN = ceil(2*edges/nv);              % avg nr of neighbors per node

for i = 1:edges                         % add all edges

  neigbors = zeros(nv, 2);              % create matrix with 2 columns
  for j = 1:nv
    neighbors(j,:) = [j,CG.node{j}.non];% col 1 = agent nr, 2= # of neigh
  end

  ids=zeros(2,1);                       % space for two ids with min # neighs.
  for k=1:2                             
    mins=find(neighbors(:,2)==min(neighbors(:,2)));    % get all min # neigh
    ids(k)=neighbors(mins(ceil(rand*length(mins))),1); % get one randomly
    neighbors(ids(k),2)=realmax;                       % erase agent
    for l=1:length(CG.node{ids(k)}.neigh)              % and its neigh
      id=CG.node{ids(k)}.neigh{l}.id;
      neighbors(id,2)=realmax;
    end
  end
  
  ok = (ids(1) ~= ids(2));              % check if not the same agent
  for l=1:length(CG.node{ids(1)}.neigh) % check if not already neighbor
    if( CG.node{ids(1)}.neigh{l}.id == ids(2) )
      ok = 0;
    end
  end
  
  if( ok == 1 )
%    fprintf( 'connect %d %d\n', ids(1), ids(2) );

    % make ids(2) neighbor of ids(1)
    non1                                 = CG.node{ids(1)}.non;
    CG.node{ids(1)}.neigh{non1+1}.id     = ids(2);          % add neighbor id
    randF                                = randn(nrA,nrA);  % init rand payoffs
    CG.node{ids(1)}.neigh{non1+1}.fij    = randF;           % set payoff struct
    CG.node{ids(1)}.non                  = non1+1;          % raise
                                                            % #
                                                            % neigh
                                                            % 
    CG.node{ids(1)}.deleted              = 0;               % raise # neigh 

    % make ids(1) neighbor of ids(2)
    non2                                 = CG.node{ids(2)}.non;      
    CG.node{ids(2)}.neigh{non2+1}.id     = ids(1);          % add neighbors id
    CG.node{ids(2)}.neigh{non2+1}.fij    = randF';          % payoff trans'd
    CG.node{ids(2)}.non                  = non2+1;          % raise
                                                            % #
                                                            % neigh
                                                            % 
    CG.node{ids(2)}.deleted              = 0;               % raise # neigh 
  else
    printf( 'already neighbor!!!' );
  end
end

% adjacency matrix
A = zeros(nv,nv);

% debug 
avg_degree=0;
for i=1:length(CG.node)                 % get total nr of neighbors
  avg_degree = avg_degree + length(CG.node{i}.neigh);
  for j=1:length(CG.node{i}.neigh)
    A(i,CG.node{i}.neigh{j}.id) = 1;
  end
end
fprintf( 'avg_degree = %f\n', avg_degree/nv );  % avg degree = #neigh/#agents

% save adjacency matrix
A
header = sprintf('%d',nv);
dlmwrite('Adj.txt',header,'');
dlmwrite('Adj.txt',A,' ','-append');
