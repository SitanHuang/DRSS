classdef TestMassGroup < matlab.unittest.TestCase
    % TestMassGroup contains tests for the MassGroup class" recalcMassGroupProperties method

    methods (Test)
        function testSingleMass(testCase)
            % Test with a single mass object
            mg = DRSS.core.obj.MassGroup("TestGroup");
            m1 = DRSS.core.obj.Mass("Mass1", "m", 100, "cgX", 10, "len", 5, "sideArea", 2);
            mg.appendChild(m1);
            testCase.verifyEqual(mg.m, 100);
            testCase.verifyEqual(mg.len, 5);
            testCase.verifyEqual(mg.cgX, 10);
            testCase.verifyEqual(mg.sideArea, 2);
        end

        function testMultipleMasses(testCase)
            % Test with multiple mass objects
            mg = DRSS.core.obj.MassGroup("TestGroup");
            m1 = DRSS.core.obj.Mass("Mass1", "m", 100, "cgX", 10, "len", 5, "sideArea", 2);
            m2 = DRSS.core.obj.Mass("Mass2", "m", 200, "cgX", 5, "len", 3, "sideArea", 1);
            m3 = DRSS.core.obj.Mass("Mass3", "m", 300, "cgX", 20, "len", 3, "sideArea", 1);
            mg.appendChild(m1);
            mg.appendChild(m2);
            mg.appendChild(m3);
            testCase.verifyEqual(mg.m, 600);
            testCase.verifyEqual(mg.len, 11);
            testCase.verifyEqual(mg.cgX, (100*10 + 200*(5+5) + 300*(20+5+3)) / 600);
            testCase.verifyEqual(mg.sideArea, 4);

            % With offsets:
            mg = DRSS.core.obj.MassGroup("TestGroup");
            m1 = DRSS.core.obj.Mass("Mass1", "m", 100, "cgX", 10, "len", 5, "sideArea", 2);
            m2 = DRSS.core.obj.Mass("Mass2", "m", 200, "cgX", 5, "len", 3, "sideArea", 1);
            m3 = DRSS.core.obj.Mass("Mass3", "m", 300, "cgX", 20, "len", 3, "sideArea", 1, "offset", -1);
            mg.appendChild(m1);
            mg.appendChild(m2);
            mg.appendChild(m3);
            testCase.verifyEqual(mg.m, 600);
            testCase.verifyEqual(mg.len, 11-1);
            testCase.verifyEqual(mg.cgX, (100*10 + 200*(5+5) + 300*(20+5+3-1)) / 600);
            testCase.verifyEqual(mg.sideArea, 4);
        end

        function testZeroMass(testCase)
            % Test adding a mass with zero properties
            mg = DRSS.core.obj.MassGroup("TestGroup");
            m1 = DRSS.core.obj.Mass("Mass1", "m", 0, "cgX", 0, "len", 0, "sideArea", 0);
            mg.appendChild(m1);
            testCase.verifyEqual(mg.m, 0);
            testCase.verifyEqual(mg.len, 0);
            testCase.verifyEqual(mg.cgX, 0);
            testCase.verifyEqual(mg.sideArea, 0);
        end
    end
end
