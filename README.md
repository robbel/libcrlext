# README #

Fork of [libcrl](http://code.google.com/p/libcrl/) to add more experimental rl algorithms, domain template and multiagent support, etc.

It is currently maintained as a private repository at https://bitbucket.org/robbel/libcrlext.

### How do I get set up? ###

* Install scons and clang++3.5 (or, alternatively, change compiler in each SConstruct file)
* lpsolve (5.5) is required to support the ALP solver. Required is the static library version that was compiled with -fPIC (e.g. `liblpsolve55-dev` in Ubuntu). *Note:* For cplex model export, lpsolve's `libxli_CPLEX` has to be compiled from source.
* Install RLGlueCore, e.g. via the version 3.04 DEB package from the [rl-glue website](https://code.google.com/p/rl-glue-ext/wiki/RLGlueCore)
* *Optional:* Install the latest version of [SPUDD](https://cs.uwaterloo.ca/~jhoey/research/spudd/) to support model file export and policy import. *Note:* on 64bit architectures, needs Makefile changes to use -fPIC switch.
* For unit tests, the [Google Test](http://code.google.com/p/googletest) framework is used (use `libgtest-dev` in Ubuntu)
* Adjust the paths in setpaths.sh to mimic your installation. Source them into the current shell via `. setpaths.sh`
* To compile, invoke `scons` in every folder, starting with `external` (the dependencies), then `libcrl` (the main library), `rlgnmn` (the network wrapper), and then the remaining ones in arbitrary order

### Contribution guidelines ###

Files are licensed under the [LGPL](http://www.gnu.org/licenses/lgpl) version 3.0, or (at your option) any later version.

### Who do I talk to? ###

The [repo owner](https://bitbucket.org/robbel) can be contacted at [robbel@gmail.com](mailto:robbel@gmail.com).