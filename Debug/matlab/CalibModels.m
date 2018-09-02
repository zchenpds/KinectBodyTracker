classdef CalibModels
    methods
        function x = getInitParams(obj)
            if obj == CalibModels.Symmetrical
                x = [pi, 1.0, 0.0];
            elseif obj == CalibModels.Asymmetrical
                x = [pi, 1.0, 1.0, 0.0];
            else
                error('Unknown model!');
            end
        end
        function tau = getTau(obj, params)
            if obj == CalibModels.Symmetrical
                tau = params(3);
            elseif obj == CalibModels.Asymmetrical
                tau = params(4);
            else
                error('Unknown model!');
            end
        end
    end
    enumeration
        Symmetrical, Asymmetrical
    end
end