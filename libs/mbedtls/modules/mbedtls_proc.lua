--[[ 
    @brief Find a related file from a list of source directories
    @param src_dir_lst list of source directories
    @param rel_path relative path to find
    @return full path if found, nil otherwise
 ]]
function _find_related_path(src_dir_lst, rel_path)
    for _, src_dir in ipairs(src_dir_lst) do
        local full_path = path.join(src_dir, rel_path)
        if os.exists(full_path) then
            return full_path
        end
    end
    return nil
end

--[[ 
    @brief Build ssl_debug_helpers_generated.c file
    @param src_dir_lst list of source directories
    @param out_path output path for generated file
 ]]
function build_ssl_helper(src_dir_lst, out_path)
end

--[[ 
    @brief Build version_features.c file
    @param src_dir_lst list of source directories
    @param out_path output path for generated file
 ]]
function build_version_features(src_dir_lst, out_path)
end

function build_errors(src_dir_lst, out_path)
end
