function [result] = ConcatenateTimeVector(matrix, newVector)
    if(size(matrix,2) > size(newVector,2))
        newVector = padarray(newVector,[0 size(matrix,2) - size(newVector,2)],'post');         
    elseif(size(matrix,2) <= size(newVector,2))
        matrix = padarray(matrix,[0 size(newVector,2) - size(matrix,2)],'post');
    end
    matrix(size(matrix,1)+1,:) = newVector;
    result = matrix;
end
