function [name, dist_km] = nearest_city(lat_deg, lon_deg)
%NEAREST_CITY Return name and distance of nearest major city.
%   [NAME, DIST_KM] = NEAREST_CITY(LAT_DEG, LON_DEG) searches a small
%   built-in list of global cities and returns the closest city name and
%   great-circle distance in kilometres. If the nearest city is more than
%   150 km away, NAME is '(none)'.

    cities = [
        struct('name','Sydney','lat',-33.8688,'lon',151.2093);
        struct('name','Melbourne','lat',-37.8136,'lon',144.9631);
        struct('name','Perth','lat',-31.9505,'lon',115.8605);
        struct('name','Adelaide','lat',-34.9285,'lon',138.6007);
        struct('name','Brisbane','lat',-27.4698,'lon',153.0251);
        struct('name','Darwin','lat',-12.4634,'lon',130.8456);
        struct('name','Hobart','lat',-42.8821,'lon',147.3272);
        struct('name','Canberra','lat',-35.2809,'lon',149.1300);
        struct('name','London','lat',51.5074,'lon',-0.1278);
        struct('name','New York','lat',40.7128,'lon',-74.0060);
        struct('name','Tokyo','lat',35.6895,'lon',139.6917);
        struct('name','Singapore','lat',1.3521,'lon',103.8198)
    ];
    R = 6371.0; % Earth radius km
    dist_km = inf; name = '(none)';
    for k = 1:numel(cities)
        c = cities(k);
        d = haversine(lat_deg, lon_deg, c.lat, c.lon, R);
        if d < dist_km
            dist_km = d;
            name = c.name;
        end
    end
    if dist_km > 150
        name = '(none)';
    end
end

function d = haversine(lat1, lon1, lat2, lon2, R)
    phi1 = deg2rad(lat1); phi2 = deg2rad(lat2);
    dphi = deg2rad(lat2 - lat1); dl = deg2rad(lon2 - lon1);
    a = sin(dphi/2).^2 + cos(phi1).*cos(phi2).*sin(dl/2).^2;
    d = 2*R*asin(sqrt(a));
end
