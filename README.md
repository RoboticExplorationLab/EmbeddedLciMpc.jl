# EmbeddedLciMpc.jl
This is a simple example that embeds Julia into C++

## Usage 
First we have to instantiate the Julia environment. Open the Julia REPL in the project directory, then run:

```
using Pkg
Pkg.activate(".")
Pkg.instantiate()
exit()
```

To create the cpp executable, run the following:
```
mkdir build 
cd build 
cmake ..
make
```
You can see the current example with: 
```
./main
```

## Issues and TODOs
1. ContactImplicitMPC uses MeshCat, which can't be built in the Docker environment without some dependencies
    * TODO: add this to Dockerfile `apt get install -y unzip`
2. Need to install Julia on Docker environment
    * TODO: add the following
    ```
    curl https://julialang-s3.julialang.org/bin/linux/x64/1.6/julia-1.6.6-linux-x86_64.tar.gz -o julia-1.6.6-linux-x86_64.tar.gz

    tar -xvzf julia-1.6.6-linux-x86_64.tar.gz

    sudo mv julia-1.6.6/ /opt/

    sudo ln -s /opt/julia-1.6.6/bin/julia /usr/local/bin/julia
    ```
3. Maybe find a way to wrap functions like this 
    ```
    https://stackoverflow.com/questions/1657883/variable-number-of-arguments-in-c
    ```
4. Multi threading works kind of funky with Julia. If you have one instance of 
   Julia running, make sure to initialize it in one thread and call any Julia 
   related function in that thread only. Otherwise it just won't work and it won't
   throw any error at you either. 
