%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Measurement Model
% new = fastMeasureModel(prev, z, tx, sz, R, enc)
% 
% new   : reweighted particles based on measurement model
% prev  : particles to be distributed
% z     : measurements of all AP distances
% tx    : transmitter locations
% sz    : noise in each measurement
% enc   : trained network
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function new = fastMeasureModel(prev, z, tx, sz, R, enc)
    global useClas numAPs hardClas
    
    DEBUG = 0;
    
    Qt = diag([10, 10]);               % initial covariance in position of landmark
    np = length(prev); new = prev;
    totalWeight = 0;
    for i = 1:np
        particle = new(i);
        
        % first see which all routers are within the sensing range
        for j = 1:numAPs
            txPos = tx(j,:);
            d = dist(txPos, particle.pose);

            % AP seen only if in Sensing Range
            if d<R
                % initialize particle if not seen before
                if isempty(find(particle.mapID==j))
                    particle.mapMu = [particle.mapMu; txPos];           % initialize mean 
                    particle.mapSigma = [particle.mapSigma; Qt];        % initialize covar
                    particle.mapID = [particle.mapID; j];               % add particle as seen
                    particle.hashMap(j) = length(particle.mapID);       % add particle to map between position vs #AP
                    particle.w = 1/np;                                  % initialize weight to 1/np
                
                % perform KF update for each landmark in map
                else
                    % get particle ID
                    id = particle.hashMap(j);
                    pos = particle.pose;
                    
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % Prediction Step
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    if id>length(particle.mapMu)
                        disp("ERROR: number of APs more than indexed!");
                        continue;
                    end
                    
                    muHat = particle.mapMu(id,:)';
                    sigHat = particle.mapSigma(2*id-1:2*id, :);
                    
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % Update Step
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    d = dist(pos,muHat);
                    
                    if useClas
                        data = [d; z(j).rssi];
                        
                        if hardClas
                            label = getLabel(data, enc);
                            if label==0
                                innovation = abs(z(j).rssi - d);
                            else
                                continue;
                            end
                        else
                            probs = enc(data);
                            innovation = probs(1)*abs(z(j).rssi - d) + probs(2)*(abs(z(j).rssi - normrnd(10,3)));
                        end
                        
                    else 
                        innovation = abs(z(j).rssi-d);
                    end
                    
                    % sensor model jacobian
                    % h = dist(particle position, estimated transmitter position)
                    den = 2*sqrt((muHat(1) - pos(1))^2 + (muHat(2) - pos(2))^2);
                    H = 2*[(muHat(1) - pos(1))/den, (muHat(2) - pos(2))/den];
                    Q = H*sigHat*H' + sz(j);
                    
                    % Kalman Gain
                    K = sigHat*H'/Q;
                    
                    % update position and covariance
                    mu = muHat + K*innovation;
                    sig = (eye(2) - K*H)*sigHat;
                    
                    particle.mapMu(id,:) = mu';
                    particle.mapSigma(2*id-1:2*id, :) = sig;
                    
                    % should all the weights from the routers be 
                    % added here or just the max weight taken?
                    particle.w = max(particle.w, sqrtm(2*pi*Q)*expm(-0.5*innovation'*(Q^-1)*innovation));
                end
                
                % update particle
                new(i) = particle;
                
            end
        end
    end
    
    % normalize particle weights
    for i = 1:np
        totalWeight = totalWeight + new(i).w;
    end
    
    % take care of 0 weights
    if totalWeight==0
        for i =1:np
            new(i).w = 1/np;
        end
    else
        for i = 1:np
            out = new(i).w/totalWeight;
            
            % just set particle weight to original if 
            % weight less than threshold
            if out<1e-15
                new(i).w = 1/np;
            else
                new(i).w = out;
            end
        end
    end
    
    if DEBUG
        wg = zeros(np,1);
        for i = 1:np
            wg(i) = new(i).w;
        end
        disp("sum:"+num2str(sum(wg))+" max: "+num2str(max(wg))+" min:"+num2str(min(wg)));
    end
    
end