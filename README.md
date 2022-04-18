# EmbeddedLciMpc.jl
This is a simple example that embed julia into C++

## Usage 
First we have to instantiate the julia environment. Open the Julia REPL in the project directory, then run:

```
using Pkg
Pkg.activate(".")
Pkg.instantiate()
exit()
```

To create the cpp exectuable, runs the following
```
mkdir build 
cd build 
cmake ..
make
```
You can see the current example with 
```
./main
```