function require_files(paths)
%REQUIRE_FILES  Assert that all given paths exist.
for i=1:numel(paths)
    if ~isfile(paths{i})
        error('Required file not found: %s', paths{i});
    end
end
end
