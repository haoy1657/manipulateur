function getIndSolCallback(hObject, eventdata)
    
    % Get the new value for q
    IndSolNew = eval(get(hObject, 'String'));
        
    if(~isempty(IndSolNew))
        if(isnumeric(IndSolNew))
            if(size(IndSolNew,1) == 1)
                assignin('base','IndSol',IndSolNew);
            end
        end
    end
end