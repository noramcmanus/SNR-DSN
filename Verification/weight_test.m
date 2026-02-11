classdef weight_test<matlab.unittest.TestCase
    methods(Test)
        function weight_test_1(TestCase)
            actual_weight=weight_func();
            TestCase.verifyLessThan(actual_weight,25000)
        end
    end
end