function valid = is_gnss_valid(gnss_location)

valid = ~isempty(gnss_location) && ~any(isnan(gnss_location));

end