function cycloid = camSVAJ(camAngles,Beta,h)
%camSVAJ Creates SVAJ plots for a barrel cam
%   cycloid follows cycloidal motion
arguments (Input)
    camAngles
    Beta
    h
end

arguments (Output)
    cycloid
end

cycloid.a = zeros(1,length(camAngles));
cycloid.v = zeros(1,length(camAngles));
cycloid.s = zeros(1,length(camAngles));
for camAngle = 1:length(camAngles)
    theta = camAngles(camAngle);
    cycloid.j(camAngle) = 4*pi^2*h/Beta^3*cos(2*pi*theta/Beta);
    cycloid.a(camAngle) = 2*pi*h/Beta^2*sin(2*pi*theta/Beta);
    cycloid.v(camAngle) = h/Beta*(1-cos(2*pi*theta/Beta));
    cycloid.s(camAngle) = h*(theta/Beta - 1/2/pi*sin(2*pi*theta/Beta));
end
end