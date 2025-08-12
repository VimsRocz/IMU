function world_map_track(lat, lon, varargin)
%WORLD_MAP_TRACK Plot a world map and overlay a lat/lon track
%   world_map_track(lat, lon, 'labels', {})

p = inputParser;
p.addParameter('labels', {});
parse(p, varargin{:});
labels = p.Results.labels;

try
    geoaxes; hold on;
    geoscatter(lat, lon, '.');
catch
    load coastlines
    plot(coastlon, coastlat, 'k'); hold on;
    plot(lon, lat, 'b');
    axis equal; xlim([-180 180]); ylim([-90 90]);
end

if ~isempty(labels)
    for i = 1:numel(labels)
        text(labels{i}.lon, labels{i}.lat, labels{i}.name);
    end
end
end
