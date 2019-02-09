%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Measurement Model
% new = measureModel(prev, z, numP, nAP, tx, sz, R, enc)
% 
% new   : reweighted particles based on measurement model
% prev  : particles to be distributed
% z     : measurements of all AP distances
% numP  : number of particles
% nAP   : number of transmitters (APs)
% tx    : transmitter locations
% sz    : noise in each measurement
% enc   : trained network
% 
% refer - http://cecas.clemson.edu/~ahoover/ece854/lecture-notes/lecture-pf.pdf
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function new = measureModel(prev, z, numP, nAP, tx, sz, R, enc)
    global useClas hardClas
        
    new = prev;
    totalWeight = 0;
    
    for i = 1:numP
        dz = zeros(nAP,1);
        % first see which all routers are within the sensing range
        for j = 1:nAP
            d = dist(tx(j,:),[prev(1,i),prev(2,i)]);
            
            % if it's in the sensing range, only then consider the AP
            if d<R
                % if classifier is used, add extra layer to updating dz
                if useClas
                    % arrange data as euclid; rssi for classifier
                    data = [d;z(j).rssi];
                    
                    % hard classification
                    % p(z|x,c) = p(z,c|x) if c = 0
                    if hardClas
                        label = getLabel(data, enc);
                        if label==0
                            dz(j) = abs(z(j).rssi-d);
                        end
                        
                    % soft classification
                    % p(z|x,c) = p(c|x)p(z,c|x) + p(~c|x)p(z,~c|x)
                    else
                        probs = enc(data);
                        dz(j) = probs(1)*abs(z(j).rssi-d) + probs(2)*abs(z(j).rssi - normrnd(10,3));
                    end
                    
                % else directly update just using the sensing range
                else
                    dz(j) = abs(z(j).rssi-d);
                end
                
            end
            
        end
        % calculate the weight as per mvnpdf for each z
        if ~isempty(dz(dz~=0))
            weight = mvnpdf(dz(dz~=0), 0, sz(dz~=0));
            if isempty(weight)
                weight = 1/numP;
            else
                new(3,i) = prev(3,i)*weight;
            end
            totalWeight = totalWeight + weight;
        end
    end
    
    % normalize the weights
    if totalWeight
        new(3,:) = new(3,:)./totalWeight;
    else % set uniform weights
        new(3,:) = 1/size(new,2);
    end

end