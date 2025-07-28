classdef SatelliteDataReader
    properties
        doppler_file_path
        ephemeris_file_path
        doppler_data
        ephemeris_df
    end

    methods
        function obj = SatelliteDataReader(doppler_file_path, ephemeris_file_path)
            obj.doppler_file_path = doppler_file_path;
            obj.ephemeris_file_path = ephemeris_file_path;
            obj.doppler_data = containers.Map('KeyType', 'char', 'ValueType', 'any');
            % obj.ephemeris_df = [];
        end

        function obj = read_doppler_data(obj)
            % Read satellite Doppler observation data from file, grouped by epoch
            fid = fopen(obj.doppler_file_path, 'r');
            if fid == -1
                error('Failed to open Doppler file: %s', obj.doppler_file_path);
            end
            current_epoch = '';
            while ~feof(fid)
                line = strtrim(fgets(fid));
                if startsWith(line, '$')
                    % Read current epoch information
                    parts = strsplit(line);
                    if length(parts) >= 10  % Check if there are enough parts
                        week = str2double(parts{2});
                        sow = str2double(parts{3});
                        sat_num = str2double(parts{4});
                        init_pos = cellfun(@str2double, parts(5:10));
                        current_epoch = sprintf('%d_%f', week, sow);  % Generate current epoch string
                        obj.doppler_data(current_epoch) = struct('satellite_count', sat_num, ...
                            'initial_position', init_pos, ...
                            'measurements', []);
                    else
                        warning('Line does not contain enough elements: %s', line);
                    end
                elseif startsWith(line, 'L') && ~isempty(current_epoch)
                    % Extract single satellite Doppler data
                    parts = strsplit(line);
                    sat_id = parts{1};
                    doppler_value = str2double(parts{2}) * 299792458 / 1.626e9;%11000000000.0;
                    % Retrieve the current data structure from the map
                    current_data = obj.doppler_data(current_epoch);
                    current_data.measurements{end+1} = {sat_id, doppler_value};  % Append measurements

                    % Update the map with the modified structure
                    obj.doppler_data(current_epoch) = current_data;
                end
            end
            fclose(fid);
        end

        function obj = read_ephemeris_data(obj)
            % 读取卫星星历数据
            ephemeris_data = {};
            fid = fopen(obj.ephemeris_file_path, 'r');
            if fid == -1
                error('无法打开星历文件: %s', obj.ephemeris_file_path);
            end

            while ~feof(fid)
                line = strtrim(fgets(fid));
                if startsWith(line, 'L')
                    parts = strsplit(line);
                    if length(parts) >= 8  % 确保有足够的部分
                        prn = parts{1};
                        pos = cellfun(@str2double, parts(2:7));
                        clk_shift = str2double(parts{8});
                        ephemeris_data = [ephemeris_data; {prn, pos, clk_shift}];  % 存储数据
                    else
                        warning('行中没有足够的元素: %s', line);
                    end
                end
            end
            fclose(fid);

            % 创建表格，确保 VariableNames 有效
            if ~isempty(ephemeris_data)
                obj.ephemeris_df = cell2table(ephemeris_data, 'VariableNames', ...
                    {'PRN', 'Pos', 'ClockShift'});
                % disp('read ephemeris:')
                % disp(obj.ephemeris_df);
            else
                error('星历数据为空，无法创建表格。');
            end
        end

        function doppler_data = get_doppler_data(obj)
            % Get Doppler data grouped by epoch
            doppler_data = obj.doppler_data;
        end

        function ephemeris_data = get_ephemeris_data(obj)
            % Get ephemeris data table
            ephemeris_data = obj.ephemeris_df;
        end

        function [measurements, init_pv] = get_measurement_for_epoch(obj, epoch_index)
            % disp('current ephemeris:')
            % disp(obj.ephemeris_df);
            % 获取指定时间段的测量数据
            epoch_keys = keys(obj.doppler_data);
            if epoch_index <= length(epoch_keys)
                epoch = epoch_keys{epoch_index};
                doppler_measurements = obj.doppler_data(epoch).measurements;
                init_pv = obj.doppler_data(epoch).initial_position;

                % 使用卫星ID获取对应的速度和位置
                satellite_data = obj.ephemeris_df;  % 确保这是一个表格
                measurements = [];
                for i = 1:length(doppler_measurements)
                    sat_id = doppler_measurements{i}{1};
                    doppler_value = doppler_measurements{i}{2};

                    % 根据 PRN 进行比较
                    if istable(satellite_data) % 确保它是一个表格
                        idx = strcmp(satellite_data.PRN, sat_id);
                    else
                        error('卫星数据不是表格类型。');
                    end

                    if any(idx)
                        satellite_info = satellite_data(idx, :);
                        % disp(satellite_info)
                        position_data = satellite_info.Pos(1,:);  % 提取位置数组
                        satellite_position = position_data(1:3);  % 前三列为位置
                        satellite_velocity = position_data(4:6);  % 后三列为速度
                        measurements = [measurements; Measurement(doppler_value, satellite_velocity, satellite_position)];
                    end
                end
            else
                error('时间段索引超出范围。');
            end
        end
    end
end
