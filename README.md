SGM
======

SGM is a highly portable geometric modeling library written in C++.
SGM supports multi-threaded execution on modern CPUs and input from standard formats like STEP and STL files. 
SGM fulfills the need for a robust and efficient geometry kernel — a need in all types of computational modeling and simulation of engineering systems.  

We expect SGM to enhance collaboration across government laboratories and universities.
We hope to improve quality and robustness of SGM by reducing defects across many use cases.

License
-------

BSD. Specifically the "new/modified BSD" license, also called the "BSD three clause license".
See [license file](https://github.com/sgmtools/sgm/blob/master/LICENSE).

Documentation
-------------

API documentation coming soon.

Goals and Capabilities
----------------------

* A reduced topological model geometric kernel for B-reps (boundary representations).
* Many operations including find closest point, ray-firing, intersections, Booleans, facetting.
* Support for the standard geometry format STEP and STL files.
* A lightweight geometry viewer based on QT. 
* Optimized multithreaded processing.
* Written in standard C++11 for portability, builds on compilers including Windows MSVC, GCC, Clang, and Intel.
* Around 95% unit test coverage of all library code.

Building the Library
-------------------- 

### Dependencies

The `sgm` library can be built with a C++11 standard compliant compiler and has no dependencies on external libraries.
The lightweight viewer application requires the [QT](https://www.qt.io/) library and [OpenGL](https://en.wikipedia.org/wiki/OpenGL).
Using the SGM build scripts requires [CMake](https://cmake.org/).

### CMake Configuration

The sgm package uses a CMake build. Here are the optional variables that may be passed to `cmake` for configuration, 
and if the variable has one, its default value.

* `CMAKE_INSTALL_PREFIX`, set a path to the location you wish to install the library after building
* `BUILD_SHARED_LIBS [OFF]`, set to `ON` to build shared as opposed to static libraries
* `BUILD_MULTITHREADED [OFF]`, set to `ON` to build with multithreading support
* `BUILD_MODEL_VIEWER [OFF]`, set to `ON` to build the QT based lightweight viewer application
* `Qt5_DIR`, set a path to the install location of the QT library (required if `BUILD_MODEL_VIEWER=ON`)

### Mac Prerequisites

* Ensure you have XCode and the [XCode Command Line Tools](https://developer.apple.com/download/more/) (Apple Developer ID required).
* Ensure you have CMake, or install CMake (using [Homebrew](https://brew.sh/) for example).

    ```brew install cmake```

* Ensure you have a compiler, preferably either:
  * Clang (from your XCode install, or llvm Homebrew),
    * If you do use llvm from Homebrew and MacOS Mojave with XCode 10.14 or later, you must complete this additional step to install standard C/C++ headers:
    
      ```open /Library/Developer/CommandLineTools/Packages/macOS_SDK_headers_for_macOS_10.14.pkg```
  
  * GCC (we recommend version 7.0+, from Homebrew for example).
  
    ```brew install gcc@7```  
  
* Optionally, if you would like to build the viewer application, install QT (download from [QT web page](https://www.qt.io/) or Homebrew).

    ```brew install qt```
    
### Linux Prerequisites

* Ensure you have CMake.
* Ensure you have a recent compiler with good support for C++11, we recommend GCC 7 or newer.
* To build the viewer application, ensure you have QT installed.
    
### Build on Linux or Mac

1. Clone the sgm git repository and make a build directory (name of your choice).

    ```
    cd sgm
    mkdir build; cd build
    ```

2. Run and configure CMake, for example, on the command line,

    ```
    cmake .. -DCMAKE_INSTALL_PREFIX=~/sgm_install -DBUILD_SHARED_LIBS=ON -DBUILD_MODEL_VIEWER=ON -DQt5_DIR=/usr/local/opt/qt/lib/cmake/Qt5
    ```

3. Run the ```make``` tool (or ```ninja```, or whatever makefile generator you used to configure CMake above) to build all targets and install.

    ```
    make
    make install
    ```
4. Optionally build and run unit tests:

    ```
    make sgm_tests
    cd bin
    ./sgm_tests
    ```

### Windows Prerequisites

* If you do not have it already, download and install [Microsoft Visual Studio 2017](https://visualstudio.microsoft.com/downloads/) (or later), the Community Edition is free.
* Download and install a version of [QT](https://www.qt.io/download) (choose the open source version if suitable).

### Build on Windows

1. Clone the sgm git repository and make a build directory (name of your choice).
2. Later versions Visual Studio support CMake projects from the "Add New Project" dialog.
   Use the dialog to setup the CMake build.
  
    a. Set relevant values for the CMake variables above, under **CMake Configuration**.
    
    b. The build targets include SGM (library), ``sgm_viewer``, and ``sgm_tests``.

Running Tests
-------------

The SGM product includes a version of the Google Test (gtest) source. 
The CMake target for running the main set of unit tests is ``sgm_tests``. 
Once the ``sgm_tests`` target is built per instructions above,
it can be executed on the command line, or in your favorite IDE. Most recent versions of C++ IDE's have support for 
discovery and running of google tests. For example, in Microsoft Visual Studio versions 2017 and later, Google Tests in
a CMake project are automatically discovered and listed in the *Test Explorer*.

