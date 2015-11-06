function env = grl_rpc_env(port, file)
%GRL_RPC_ENV RPC interface to grl environment.
%   ENV = GRL_RPC_ENV(PORT) connects to a grl deployer running on
%   the local host on port PORT.
%   ENV = GRL_RPC_ENV(PORT, HOST) connects to a grl deployer running on
%   the remote host HOST on port PORT.
%   ENV = GRL_RPC_ENV(PORT, FILE) starts a new grl deployer on localhost
%   on port PORT and connects to it.
%
%   O = ENV.START() resets the environment, returning the initial
%   observation.
%   [O, R, T, TAU] = ENV.STEP(A) executes action A and returns the new
%   observation O, reward R, termination T and elapsed time TAU.
%   ENV.FINI() closes the connection, terminating the deployer.
%
%   AUTHOR:
%       Wouter Caarls <wouter@caarls.org>

    if nargin < 2
        % grld already running on localhost
        fd = tcpip('localhost', port, 'ByteOrder', 'littleEndian');
    elseif any(strfind(file, '.yaml'))
        % Start new grld instance on localhost
        state = system(['echo "experiment port=' num2str(port) '" | grld ' file ' -c &']);

        if state ~= 0
            error('Failed to run grl deployer');
        end

        fd = tcpip('localhost', port, 'ByteOrder', 'littleEndian');
    else
        % grld already running on remote host
        fd = tcpip(file, port, 'ByteOrder', 'littleEndian');
    end

    tic;
    opened = 0;
    while ~opened
        try
            fopen(fd)
            opened = 1;
        catch
            if toc > 1
                error('Failed to connect to grl deployer')
            end
        end
    end
    
    env.start = @start;
    env.step = @step;
    env.fini = @fini;

    function [o, r, t, tau] = step(a)
        fwrite(fd, numel(a));
        if numel(a) > 0
            fwrite(fd, a, 'double');
        end
        n = fread(fd, 1);
        o = fread(fd, n, 'double');
        r = fread(fd, 1, 'double');
        t = fread(fd, 1);
        tau = fread(fd, 1, 'double');
    end

    function o = start()
        o = step([]);
    end

    function fini()
        fclose(fd);
        delete(fd);
    end

end
