function [ Q ] = mJacobianoDir(X)
J = [                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      1/3,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        0,                                                                                                                                                                                                                                                                      1/3,                                                                                                                                                                                                                                                                       0,                                                                                                                                                                                                                                                                     1/3,                                                                                                                                                                                                                                                                        0;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      1/3,                                                                                                                                                                                                                                                                        0,                                                                                                                                                                                                                                                                     1/3,                                                                                                                                                                                                                                                                       0,                                                                                                                                                                                                                                                                      1/3;
                                                                                                                                                                                                                                                                                                                                                            (9*((2*imag(X(3)))/9 - (4*imag(X(1)))/9 + (2*imag(X(5)))/9 - (4*real(X(2)))/9 + (2*real(X(4)))/9 + (2*real(X(6)))/9))/((imag(X(3)) - 2*imag(X(1)) + imag(X(5)) - 2*real(X(2)) + real(X(4)) + real(X(6)))^2 + (2*imag(X(2)) - imag(X(4)) - imag(X(6)) - 2*real(X(1)) + real(X(3)) + real(X(5)))^2),                                                                                                                                                                                                                                                                                                                                                           -(9*((4*imag(X(2)))/9 - (2*imag(X(4)))/9 - (2*imag(X(6)))/9 - (4*real(X(1)))/9 + (2*real(X(3)))/9 + (2*real(X(5)))/9))/((imag(X(3)) - 2*imag(X(1)) + imag(X(5)) - 2*real(X(2)) + real(X(4)) + real(X(6)))^2 + (2*imag(X(2)) - imag(X(4)) - imag(X(6)) - 2*real(X(1)) + real(X(3)) + real(X(5)))^2),                           -(9*(imag(X(3))/9 - (2*imag(X(1)))/9 + imag(X(5))/9 - (2*real(X(2)))/9 + real(X(4))/9 + real(X(6))/9))/((imag(X(3)) - 2*imag(X(1)) + imag(X(5)) - 2*real(X(2)) + real(X(4)) + real(X(6)))^2 + (2*imag(X(2)) - imag(X(4)) - imag(X(6)) - 2*real(X(1)) + real(X(3)) + real(X(5)))^2),                           (9*((2*imag(X(2)))/9 - imag(X(4))/9 - imag(X(6))/9 - (2*real(X(1)))/9 + real(X(3))/9 + real(X(5))/9))/((imag(X(3)) - 2*imag(X(1)) + imag(X(5)) - 2*real(X(2)) + real(X(4)) + real(X(6)))^2 + (2*imag(X(2)) - imag(X(4)) - imag(X(6)) - 2*real(X(1)) + real(X(3)) + real(X(5)))^2),                          -(9*(imag(X(3))/9 - (2*imag(X(1)))/9 + imag(X(5))/9 - (2*real(X(2)))/9 + real(X(4))/9 + real(X(6))/9))/((imag(X(3)) - 2*imag(X(1)) + imag(X(5)) - 2*real(X(2)) + real(X(4)) + real(X(6)))^2 + (2*imag(X(2)) - imag(X(4)) - imag(X(6)) - 2*real(X(1)) + real(X(3)) + real(X(5)))^2),                            (9*((2*imag(X(2)))/9 - imag(X(4))/9 - imag(X(6))/9 - (2*real(X(1)))/9 + real(X(3))/9 + real(X(5))/9))/((imag(X(3)) - 2*imag(X(1)) + imag(X(5)) - 2*real(X(2)) + real(X(4)) + real(X(6)))^2 + (2*imag(X(2)) - imag(X(4)) - imag(X(6)) - 2*real(X(1)) + real(X(3)) + real(X(5)))^2);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              (X(1) - X(3))/((X(1) - X(3))^2 + (X(2) - X(4))^2)^(1/2),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              (X(2) - X(4))/((X(1) - X(3))^2 + (X(2) - X(4))^2)^(1/2),                                                                                                                                                                                                                             -(X(1) - X(3))/((X(1) - X(3))^2 + (X(2) - X(4))^2)^(1/2),                                                                                                                                                                                                                            -(X(2) - X(4))/((X(1) - X(3))^2 + (X(2) - X(4))^2)^(1/2),                                                                                                                                                                                                                                                                       0,                                                                                                                                                                                                                                                                        0;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              (X(1) - X(5))/((X(1) - X(5))^2 + (X(2) - X(6))^2)^(1/2),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              (X(2) - X(6))/((X(1) - X(5))^2 + (X(2) - X(6))^2)^(1/2),                                                                                                                                                                                                                                                                        0,                                                                                                                                                                                                                                                                       0,                                                                                                                                                                                                                            -(X(1) - X(5))/((X(1) - X(5))^2 + (X(2) - X(6))^2)^(1/2),                                                                                                                                                                                                                             -(X(2) - X(6))/((X(1) - X(5))^2 + (X(2) - X(6))^2)^(1/2);
 ((2*X(3) - 4*X(1) + 2*X(5))/(2*((X(1) - X(3))^2 + (X(2) - X(4))^2)^(1/2)*((X(1) - X(5))^2 + (X(2) - X(6))^2)^(1/2)) + ((2*X(1) - 2*X(3))*((X(1) - X(3))^2 + (X(1) - X(5))^2 + (X(2) - X(4))^2 + (X(2) - X(6))^2 - (X(3) - X(5))^2 - (X(4) - X(6))^2))/(4*((X(1) - X(3))^2 + (X(2) - X(4))^2)^(3/2)*((X(1) - X(5))^2 + (X(2) - X(6))^2)^(1/2)) + ((2*X(1) - 2*X(5))*((X(1) - X(3))^2 + (X(1) - X(5))^2 + (X(2) - X(4))^2 + (X(2) - X(6))^2 - (X(3) - X(5))^2 - (X(4) - X(6))^2))/(4*((X(1) - X(3))^2 + (X(2) - X(4))^2)^(1/2)*((X(1) - X(5))^2 + (X(2) - X(6))^2)^(3/2)))/(1 - ((X(1) - X(3))^2 + (X(1) - X(5))^2 + (X(2) - X(4))^2 + (X(2) - X(6))^2 - (X(3) - X(5))^2 - (X(4) - X(6))^2)^2/(4*((X(1) - X(3))^2 + (X(2) - X(4))^2)*((X(1) - X(5))^2 + (X(2) - X(6))^2)))^(1/2), ((2*X(4) - 4*X(2) + 2*X(6))/(2*((X(1) - X(3))^2 + (X(2) - X(4))^2)^(1/2)*((X(1) - X(5))^2 + (X(2) - X(6))^2)^(1/2)) + ((2*X(2) - 2*X(4))*((X(1) - X(3))^2 + (X(1) - X(5))^2 + (X(2) - X(4))^2 + (X(2) - X(6))^2 - (X(3) - X(5))^2 - (X(4) - X(6))^2))/(4*((X(1) - X(3))^2 + (X(2) - X(4))^2)^(3/2)*((X(1) - X(5))^2 + (X(2) - X(6))^2)^(1/2)) + ((2*X(2) - 2*X(6))*((X(1) - X(3))^2 + (X(1) - X(5))^2 + (X(2) - X(4))^2 + (X(2) - X(6))^2 - (X(3) - X(5))^2 - (X(4) - X(6))^2))/(4*((X(1) - X(3))^2 + (X(2) - X(4))^2)^(1/2)*((X(1) - X(5))^2 + (X(2) - X(6))^2)^(3/2)))/(1 - ((X(1) - X(3))^2 + (X(1) - X(5))^2 + (X(2) - X(4))^2 + (X(2) - X(6))^2 - (X(3) - X(5))^2 - (X(4) - X(6))^2)^2/(4*((X(1) - X(3))^2 + (X(2) - X(4))^2)*((X(1) - X(5))^2 + (X(2) - X(6))^2)))^(1/2), -((X(2) - X(4))*(X(1)*X(4) - X(2)*X(3) - X(1)*X(6) + X(2)*X(5) + X(3)*X(6) - X(4)*X(5)))/(((X(1) - X(3))^2 + (X(2) - X(4))^2)^(3/2)*((X(1) - X(5))^2 + (X(2) - X(6))^2)^(1/2)*(1 - (X(1)*X(3) + X(1)*X(5) + X(2)*X(4) + X(2)*X(6) - X(3)*X(5) - X(4)*X(6) - X(1)^2 - X(2)^2)^2/(((X(1) - X(3))^2 + (X(2) - X(4))^2)*((X(1) - X(5))^2 + (X(2) - X(6))^2)))^(1/2)), ((X(1) - X(3))*(X(1)*X(4) - X(2)*X(3) - X(1)*X(6) + X(2)*X(5) + X(3)*X(6) - X(4)*X(5)))/(((X(1) - X(3))^2 + (X(2) - X(4))^2)^(3/2)*((X(1) - X(5))^2 + (X(2) - X(6))^2)^(1/2)*(1 - (X(1)*X(3) + X(1)*X(5) + X(2)*X(4) + X(2)*X(6) - X(3)*X(5) - X(4)*X(6) - X(1)^2 - X(2)^2)^2/(((X(1) - X(3))^2 + (X(2) - X(4))^2)*((X(1) - X(5))^2 + (X(2) - X(6))^2)))^(1/2)), ((X(2) - X(6))*(X(1)*X(4) - X(2)*X(3) - X(1)*X(6) + X(2)*X(5) + X(3)*X(6) - X(4)*X(5)))/(((X(1) - X(3))^2 + (X(2) - X(4))^2)^(1/2)*((X(1) - X(5))^2 + (X(2) - X(6))^2)^(3/2)*(1 - (X(1)*X(3) + X(1)*X(5) + X(2)*X(4) + X(2)*X(6) - X(3)*X(5) - X(4)*X(6) - X(1)^2 - X(2)^2)^2/(((X(1) - X(3))^2 + (X(2) - X(4))^2)*((X(1) - X(5))^2 + (X(2) - X(6))^2)))^(1/2)), -((X(1) - X(5))*(X(1)*X(4) - X(2)*X(3) - X(1)*X(6) + X(2)*X(5) + X(3)*X(6) - X(4)*X(5)))/(((X(1) - X(3))^2 + (X(2) - X(4))^2)^(1/2)*((X(1) - X(5))^2 + (X(2) - X(6))^2)^(3/2)*(1 - (X(1)*X(3) + X(1)*X(5) + X(2)*X(4) + X(2)*X(6) - X(3)*X(5) - X(4)*X(6) - X(1)^2 - X(2)^2)^2/(((X(1) - X(3))^2 + (X(2) - X(4))^2)*((X(1) - X(5))^2 + (X(2) - X(6))^2)))^(1/2))];

Q = J*X;
end

