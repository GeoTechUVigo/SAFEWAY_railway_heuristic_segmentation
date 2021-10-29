function [status] = RecordError(ME)
% Record some errors.

% container to save the error
status = containers.Map;
% error stack withour the file path and message
error_message = [];
error_message.name = ME.stack.name;
error_message.line = ME.stack.line;
error_message.message = ME.message;
% save in the structure
status("error") = error_message;

end

