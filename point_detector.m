%*************************************************************************
%  File........: point_detector.m
%  Author(s)...: JMRMEDEV
%  URL(s)......: https://github.com/JMRMEDEV
%  Device(s)...: Any
%  Compiler....: 
%  Description.: Picture Analyzer for getting coordinated points
%  Date........: 07.05.20
%  Version.....: 0.0.4
%  Notes: This script is implemented by using Peter Corke's Machine
%  vision toolbox. I just made a functionality meant to get points
%  from any picture with defined contours in Black and White for their
%  use in other Matlab scripts.
%*************************************************************************

function pointed_image = point_detector(picture_location)

    % Read the picture
    picture = iread(picture_location);
    points_count = 0;
    coordinates = [];

    picture_display = figure('Name', 'Input Picture');
    hold on;

    % Reduces the size in ratio 10:1 for an easier
    % point detection. In somehow, converts pixels
    % to points.
    smaller = idecimate(picture, 7);

    [height,width,extra] = size(smaller);
    points_exist = [height,width];

    for i1 = 1:height
        for i2 = 1:width
            % If the color is black alike, saves the location
            if smaller(i1,i2) < 160
                if (i1 > 1 && i2 > 1) && i2 < 104
                    points_exist(i1,i2) = 1;
                    points_count = points_count + 1;
                    coordinates(points_count,1) = i1;
                    coordinates(points_count,2) = i2;
                end
            end
            % Otherwise, determines is not an important pixel
            if picture(i1,i2) ~= 0 
                points_exist(i1,i2) = 0;
            end
        end
    end

%    Constructs an offset in the x-axis. This was used for a specific 
   % application, may be omitted for different usages.
    for n1 = 1:length(coordinates)
        coordinates(n1,1) = coordinates(n1,1) - 50;
        coordinates(n1,2) = coordinates(n1,2) + 30;
        xlim([-100 100]);
        plot(coordinates(n1,1),coordinates(n1,2), 'o');
    end
    
    % Returns the matrix of n-rows and two columns
    pointed_image = coordinates;
end
