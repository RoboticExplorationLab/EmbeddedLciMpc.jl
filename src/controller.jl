function add1(a)
    return a + 1
end

function times2array(a)
    return a .* 2.0 
end

function findEigs(A)
    result = eigvals(A)
    return result
end 
