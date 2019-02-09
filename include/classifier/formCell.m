% Puts the inputs in a cell array
function Xenc = formCell(X)
    n = length(X);
    
    for i=1:n
        cel = {[X(i,1), X(i,2)]};
        Xenc(i) = cel;
    end
    
end