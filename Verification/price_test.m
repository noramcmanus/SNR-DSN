classdef price_test<matlab.unittest.TestCase
    methods(Test)
        function price_test_1(TestCase)
            actual_price=price_func();
            TestCase.verifyLessThan(actual_price,25000)
        end
    end
end