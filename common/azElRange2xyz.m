function [ p_k_j_k, H] = azElRange2xyz( azElRange , unit )
%AZELRANGE2XYZ compute a point in the sensor frame given a spherical
% coordinate measurement.
%
% Input:
% azElRange : a 3x1 column [azimuth; elevation; range];
% unit : string specifying angle units. Either 'deg' or 'rad'
% 
% Output:
% p_k_j_k : the point [x;y;z] expressed in and with respect to the sensor
%           frame

if nargin < 2
    unit = 'rad';
    warning('azElRange2xyz: no angular unit specified, using rad');
end

if strcmp(unit,'deg')
    az = (pi/180).*azElRange(1);
    el = (pi/180).*azElRange(2);
elseif strcmp(unit,'rad')
    az = azElRange(1);
    el = azElRange(2);
else
    fprintf('Warning: input angles assumed to be in degrees.\n');
end
    
r = azElRange(3);
% d = r * cos(az);
% 
% x = d * cos(el);
% y = r * sin(az);
% z = d * sin(el);

% p_k_j_k = [x;y;z];

%Create keypoint
x = r * cos(az) * cos(el);
y = r * sin(az) * cos(el);
z = r * sin(el);

p_k_j_k = [x;y;z];

if nargout == 2
    H = [-r*sin(az)*cos(el) -r*cos(az)*sin(el)    cos(az)*cos(el);...
         cos(az)*r          0                       sin(az);...
         -r*sin(az)*sin(el) r*cos(az)*cos(el)    cos(az)*sin(el)];
end

end

