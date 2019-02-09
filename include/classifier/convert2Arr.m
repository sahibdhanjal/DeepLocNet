% converts cell data structure to array
function X = convert2Arr(Xin)
    X = zeros(length(Xin{1}),length(Xin));
    
    for i=1:numel(Xin)
        X(:,i) = Xin{i}(:);
    end
end