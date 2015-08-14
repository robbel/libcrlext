# README #

Fork of [libcrl](http://code.google.com/p/libcrl/) to add more experimental rl algorithms, domain template and multiagent support, etc.

It is currently maintained as a private repository at https://bitbucket.org/robbel/libcrlext.

### How do I get set up? ###

* Install scons, boost, gsl, zlib, and clang++3.5 (or, alternatively, change compiler in each SConstruct file)
* *Deprecated: lpsolve (5.5) is required to support the ALP solver. Required is the static library version that was compiled with -fPIC (e.g. `liblpsolve55-dev` in Ubuntu).* *Note:* *For cplex model export, lpsolve's `libxli_CPLEX` has to be compiled from source.*
* Instead of lpsolve, we now use Gurobi as the ALP solver. An academic license can be obtained from the [Gurobi website](http://www.gurobi.com)
* Install RLGlueCore, e.g. via the version 3.04 DEB package from the [rl-glue website](https://code.google.com/p/rl-glue-ext/wiki/RLGlueCore)
* For approximate inference support in larger ALPs, the latest development version of [libDAI](https://staff.fnwi.uva.nl/j.m.mooij/libDAI/) from [git](http://git.tuebingen.mpg.de/libdai.git) is required. Our extensions (*max-plus*) currently require an additional patch before compilation, see `libdai.git.patch` in the `external` directory.
* *Optional:* Install the latest version of [SPUDD](https://cs.uwaterloo.ca/~jhoey/research/spudd/) to support model file export and policy import. *Note:* on 64bit architectures, needs Makefile changes to use -fPIC switch.
* For unit tests, the [Google Test](http://code.google.com/p/googletest) framework is used (use `libgtest-dev` in Ubuntu). *Note:* Some versions of Ubuntu require an additional compilation step, see [here](http://askubuntu.com/questions/145887/why-no-library-files-installed-for-google-test).
* Adjust the paths in setpaths.sh to mimic your installation. Source them into the current shell via `. setpaths.sh`
* To compile, invoke `scons` in every folder, starting with `external` (the dependencies, which need to be installed), then `libcrl` (the main library), `rlgnmn` (the network wrapper), and then the remaining ones in arbitrary order

### Contribution guidelines ###

Files are licensed under the [LGPL](http://www.gnu.org/licenses/lgpl) version 3.0, or (at your option) any later version.

### Who do I talk to? ###

The [repo owner](https://bitbucket.org/robbel) can be contacted at [robbel@gmail.com](mailto:robbel@gmail.com).