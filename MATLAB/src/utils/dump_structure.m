function dump_structure(varName, varValue)
%DUMP_STRUCTURE Print basic details about a variable.

fprintf('--- Dump of variable: %s ---\n', varName);

if isstruct(varValue)
    flds = fieldnames(varValue);
    for k = 1:numel(flds)
        fldVal = varValue.(flds{k});
        sz = size(fldVal);
        fprintf('  %s: %s [%s]\n', ...
            flds{k}, class(fldVal), num2str(sz));
    end
elseif isnumeric(varValue) || islogical(varValue) || ischar(varValue)
    sz = size(varValue);
    fprintf('  %s: %s [%s]\n', varName, class(varValue), num2str(sz));
else
    disp(varValue);
end
end
