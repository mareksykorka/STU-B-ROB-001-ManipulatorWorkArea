function outputMatrix = appRobTranslate(~, axis, d)%, inputMatrix)
    translationMatrix = eye(4);

    switch axis
        case 'x'
            translationMatrix(1,4) = d;
        case 'y'
            translationMatrix(2,4) = d;
        case 'z'
            translationMatrix(3,4) = d;
    end

    outputMatrix = translationMatrix;% * inputMatrix;
end