function ros2bag_matlab(varargin)
%ROS2BAG_MATLAB Analyze one Tianbot drift rosbag from the MATLAB command line.
%
% Command form (relative paths are resolved from the MATLAB current folder):
%   ros2bag_matlab ../ros2_bag/rosbag2_2026_07_21-17_16_51
%
% Absolute paths are also accepted. Results are written to
% BAG_DIRECTORY/analysis_4wid/report.html.

if numel(varargin) ~= 1
    error('ros2bag_matlab:InvalidInput', ...
        ['Please specify exactly one rosbag directory, for example:' newline ...
         '  ros2bag_matlab ../ros2_bag/rosbag2_2026_07_21-17_16_51']);
end

bagArgument = varargin{1};
if ~(ischar(bagArgument) || (isstring(bagArgument) && isscalar(bagArgument)))
    error('ros2bag_matlab:InvalidInput', ...
        'The rosbag path must be a character vector or string scalar.');
end

bagPath = strtrim(char(bagArgument));
if isempty(bagPath)
    error('ros2bag_matlab:InvalidBagPath', ...
        'The rosbag directory path cannot be empty.');
end

toolsDirectory = fileparts(mfilename('fullpath'));
workspaceRoot = fileparts(toolsDirectory);
bagDirectory = resolveFromCurrentFolder(bagPath);
analyzer = fullfile(workspaceRoot, 'tools', 'analyze_drift_rosbag.py');
controllerYaml = fullfile(workspaceRoot, 'src', 'tianbot_drift_test', ...
    'config', 'drift_4wid_speed_yaw_pid.yaml');
outputDirectory = fullfile(bagDirectory, 'analysis_4wid');

if ~isfolder(bagDirectory)
    error('ros2bag_matlab:BagNotFound', ...
        'Rosbag directory not found: %s', bagDirectory);
end
if isempty(dir(fullfile(bagDirectory, '*.db3')))
    error('ros2bag_matlab:DatabaseNotFound', ...
        'No .db3 file was found in: %s', bagDirectory);
end
if ~isfile(analyzer)
    error('ros2bag_matlab:AnalyzerNotFound', ...
        'Analysis tool not found: %s', analyzer);
end
if ~isfile(controllerYaml)
    error('ros2bag_matlab:ConfigNotFound', ...
        '4WID controller YAML not found: %s', controllerYaml);
end

pythonCommand = findPythonCommand();
command = sprintf('%s -B %s %s --recorded-yaml %s --output %s', ...
    pythonCommand, shellQuote(analyzer), shellQuote(bagDirectory), ...
    shellQuote(controllerYaml), shellQuote(outputDirectory));

fprintf('Analyzing rosbag: %s\n', bagDirectory);
[status, commandOutput] = system(command);
if ~isempty(commandOutput)
    fprintf('%s', commandOutput);
    if commandOutput(end) ~= newline
        fprintf('\n');
    end
end
if status ~= 0
    error('ros2bag_matlab:AnalysisFailed', ...
        'Rosbag analysis failed with exit code %d.', status);
end

fprintf('Analysis complete.\n');
fprintf('HTML report: %s\n', fullfile(outputDirectory, 'report.html'));
fprintf('Markdown summary: %s\n', fullfile(outputDirectory, 'summary.md'));
end


function absolutePath = resolveFromCurrentFolder(inputPath)
% Resolve relative input against pwd while preserving absolute paths.
if ispc
    isAbsolute = ~isempty(regexp(inputPath, ...
        '^(?:[A-Za-z]:[\\/]|[\\/]{2})', 'once'));
else
    isAbsolute = startsWith(inputPath, filesep);
end
if isAbsolute
    absolutePath = inputPath;
else
    absolutePath = fullfile(pwd, inputPath);
end
end


function command = findPythonCommand()
override = strtrim(getenv('DRIFT_ANALYZER_PYTHON'));
candidates = {};
if ~isempty(override)
    if isfile(override)
        candidates{end + 1} = shellQuote(override);
    else
        candidates{end + 1} = override;
    end
end

if ispc
    knownPython = ['D:' filesep 'Program Files' filesep ...
        'CarSim2022.1_Prog' filesep 'Programs' filesep 'Python' filesep ...
        'Python64' filesep 'python.exe'];
    if isfile(knownPython)
        candidates{end + 1} = shellQuote(knownPython);
    end
    candidates = [candidates, {'py -3', 'python', 'python3'}];
else
    candidates = [candidates, {'python3', 'python'}];
end

for index = 1:numel(candidates)
    candidate = candidates{index};
    [status, ~] = system(sprintf('%s --version', candidate));
    if status == 0
        command = candidate;
        return;
    end
end

error('ros2bag_matlab:PythonNotFound', ...
    ['No usable Python interpreter was found. Set DRIFT_ANALYZER_PYTHON ' ...
     'to the full path of Python, then run the command again.']);
end


function quoted = shellQuote(value)
quoted = ['"' char(value) '"'];
end
