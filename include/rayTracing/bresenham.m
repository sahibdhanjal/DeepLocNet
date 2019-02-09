% Copyright (c) 2010, Aaron  Wetzler
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are
% met:
% 
%     * Redistributions of source code must retain the above copyright
%       notice, this list of conditions and the following disclaimer.
%     * Redistributions in binary form must reproduce the above copyright
%       notice, this list of conditions and the following disclaimer in
%       the documentation and/or other materials provided with the distribution
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.
% Matlab optmized version of Bresenham line algorithm. No loops.
% Format:
%               [x y]=bham(x1,y1,x2,y2)
%
% Input:
%               (x1,y1): Start position
%               (x2,y2): End position
%
% Output:
%               x y: the line coordinates from (x1,y1) to (x2,y2)
%
% Usage example:
%               [x y]=bham(1,1, 10,-5);
%               plot(x,y,'or');

function [x,y]=bresenham(x1,y1,x2,y2)
    x1=round(x1); x2=round(x2);
    y1=round(y1); y2=round(y2);
    dx=abs(x2-x1);
    dy=abs(y2-y1);
    steep=abs(dy)>abs(dx);
    if steep t=dx;dx=dy;dy=t; end

    %The main algorithm goes here.
    if dy==0 
        q=zeros(dx+1,1);
    else
        q=[0;diff(mod([floor(dx/2):-dy:-dy*dx+floor(dx/2)]',dx))>=0];
    end
    if steep
        if y1<=y2 
            y=[y1:y2]'; 
        else
            y=[y1:-1:y2]'; 
        end
        if x1<=x2 
            x=x1+cumsum(q);
        else
            x=x1-cumsum(q); 
        end
    else
        if x1<=x2 
            x=[x1:x2]'; 
        else
            x=[x1:-1:x2]'; 
        end
        if y1<=y2 
            y=y1+cumsum(q);
        else
            y=y1-cumsum(q);
        end
    end
end