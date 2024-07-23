% Generate points on the circle
theta = deg2rad(15:30:345);
xz = [cos(theta)' sin(theta)'];
xz1 = 0.00585817630365063*xz;
xz2 = 0.00778237301961355*xz;
xyz = single([xz1(:,1) repmat(-0.0203041079145141,12,1) xz1(:,2) 
        xz2(:,1) repmat(-0.00719058429502904,12,1) xz2(:,2)
        xz2(:,1) repmat(0.00719058429502904,12,1) xz2(:,2) 
        xz1(:,1) repmat(0.0203041079145141,12,1) xz1(:,2)]); % vertices

theta = deg2rad(18:36:342);
xz = single(0.07988874/2*[cos(theta)' sin(theta)']); % translations
xz(abs(xz)<1e-9)=0;

theta = deg2rad(90-(0:60:300));
[cos(theta)' sin(theta)']