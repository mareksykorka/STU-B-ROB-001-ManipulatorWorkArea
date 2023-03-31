function outputMatrix = rob_rotate(axis, type, rotation)
    rotationMatrix = eye(4);

    switch type
        case 'deg'
            rad = deg2rad(rotation);
        case 'rad'
            rad = rotation;
        otherwise
            rad = deg2rad(rotation);
    end

    switch axis
        case 'x'
            rotationMatrix(2,2) = cos(rad);
            rotationMatrix(2,3) = -sin(rad);
            rotationMatrix(3,2) = sin(rad);
            rotationMatrix(3,3) = cos(rad);
        case 'y'
             rotationMatrix(1,1) = cos(rad);
            rotationMatrix(1,3) = sin(rad);
            rotationMatrix(3,1) = -sin(rad);
            rotationMatrix(3,3) = cos(rad);
        case 'z'
            rotationMatrix(1,1) = cos(rad);
            rotationMatrix(1,2) = -sin(rad);
            rotationMatrix(2,1) = sin(rad);
            rotationMatrix(2,2) = cos(rad);
    end
    
     outputMatrix = rotationMatrix;
end
