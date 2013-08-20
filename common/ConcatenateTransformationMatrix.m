function [result] = ConcatenateTransformationMatrix(matrix,newMatrix)
    if(size(matrix,3) <= size(newMatrix,3))
        matrix = padarray(matrix,[0,0,size(newMatrix,3) - size(matrix,3)],'post');
    elseif(size(matrix,3) > size(newMatrix,3))
        newMatrix = padarray(newMatrix,[0,0,size(matrix,3) - size(newMatrix,3)],'post');
    end
    result = [matrix;newMatrix];
end