function rob_sphere(~, radius, base)
    % Make unit sphere
    [x,y,z] = sphere;
    % Scale to desire radius.
    x = x * radius;
    y = y * radius;
    z = z * radius;

    surf(app.UIAxes,x+base(1),y+base(2),z+base(3),'EdgeColor','black','FaceColor','black')
end