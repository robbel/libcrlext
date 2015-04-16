# README #

Fork of [libcrl](http://code.google.com/p/libcrl/) to add more experimental rl algorithms, domain template and multiagent support, etc.

It is currently maintained as a private repository at https://bitbucket.org/robbel/libcrlext.

### How do I get set up? ###

* Install clang++3.5 (or, alternatively, change compiler in each SConstruct file)
* Compile and install the external dependencies in `external`
* lpsolve (5.5) is required to support the ALP solver. Required is the static library version that was compiled with -fPIC (e.g. `liblpsolve55-dev` in Ubuntu)
* Install RLGlueCore, e.g. via the version 3.04 DEB package from the [rl-glue website](https://code.google.com/p/rl-glue-ext/wiki/RLGlueCore)
* For unit tests, the [Google Test](http://code.google.com/p/googletest) framework is used (`libgtest-dev` in Ubuntu)
* Adjust the paths in setpaths.sh to mimic your installation. Source them into the current shell via `. setpaths.sh`

### Contribution guidelines ###

Files are licensed under the [LGPL](http://www.gnu.org/licenses/lgpl) version 3.0, or (at your option) any later version.

### Who do I talk to? ###

The [repo owner](https://bitbucket.org/robbel) can be contacted at [robbel@gmail.com](mailto:robbel@gmail.com).