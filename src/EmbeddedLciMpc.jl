module EmbeddedLciMpc

using Rotations
using LinearAlgebra

function add1(a)
    return a + 1
end

function times2array(a)
    return a .* 2.0 
end

function findEigs(A)
    println(A)
    display(eigvals(A))
    return eigvals(A)
end 

end # module
