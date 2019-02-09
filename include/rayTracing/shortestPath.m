% Interactive Image Shortest Path on an Image 
% Author: Salaheddin Hosseinzadeh
% Created on: 19/ Feb / 2016
% Last Rev: 
% Notes:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ShortestPath_V1() gets a binary image and determines the shortest path between two
% points on the image using distance transform. 
% Path is defined as the WHITE pixels between the two points!
% 
% function [path] = ShortestPath_V1(image)
% 
% path      : Binary image. showing the shortest path wit 1s
% I         : Binary image, where shortest path to be found. (Path = WHITE pixel)
% varargin  : String, 'demo' shows the path
% http://blogs.mathworks.com/steve/2011/12/13/exploring-shortest-paths-part-5/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [path] = shortestPath(I,varargin)
    if numel(I(1,1,:)) == 3
        I = rgb2gray(I); % converst to grayscale in case of an RGB image
    end    

    % ID = imdilate(I < (max(max(I))./2), ones(7,7)); % complementing the image and then dilating it
    ID = (I < (max(max(I))./2));


    if isempty(varargin)
        figure
        imshow(~ID)
        [C,R] = ginput(2);
    else
        C = varargin{1};
        R = varargin{2};
    end

    D1 = bwdistgeodesic(~ID, round(C(1,1)), round(R(1,1)), 'quasi-euclidean');
    D2 = bwdistgeodesic(~ID, round(C(2,1)), round(R(2,1)), 'quasi-euclidean');

    D = D1 + D2;
    D = round(D * 8) / 8;

    D(isnan(D)) = inf;
    paths = imregionalmin(D);

    path = bwmorph(paths, 'thin', inf);
end