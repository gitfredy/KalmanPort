function [ covEqCalc ] = calculateCovEquation( A, B )
%CALCULATECOVEQUATION Summary of this function goes here
%   For use with Eq 70 of Kraft Paper
%   Computed as the (weighted/averaged) outer product of the transformed
%   points, vectors in input matrices.


mat1 = A(:,1)*B(:,1)';
mat2 = A(:,2)*B(:,2)';
mat3 = A(:,3)*B(:,3)';
mat4 = A(:,4)*B(:,4)';
mat5 = A(:,5)*B(:,5)';
mat6 = A(:,6)*B(:,6)';
mat7 = A(:,7)*B(:,7)';
mat8 = A(:,8)*B(:,8)';
mat9 = A(:,9)*B(:,9)';
mat10 = A(:,10)*B(:,10)';
mat11 = A(:,11)*B(:,11)';
mat12 = A(:,12)*B(:,12)';
mat13 = A(:,13)*B(:,13)';
mat14 = A(:,14)*B(:,14)';
mat15 = A(:,15)*B(:,15)';
mat16 = A(:,16)*B(:,16)';
mat17 = A(:,17)*B(:,17)';
mat18 = A(:,18)*B(:,18)';
mat19 = A(:,19)*B(:,19)';
mat20 = A(:,20)*B(:,20)';
mat21 = A(:,21)*B(:,21)';
mat22 = A(:,22)*B(:,22)';
mat23 = A(:,23)*B(:,23)';
mat24 = A(:,24)*B(:,24)';
mat25 = A(:,25)*B(:,25)';
mat26 = A(:,26)*B(:,26)';
mat27 = A(:,27)*B(:,27)';
mat28 = A(:,28)*B(:,28)';


covEqCalc = (mat1 + mat2 + mat3 + mat4 + mat5 + mat6 + mat7 + mat8 + mat9 + mat10 + mat11 + mat12 + ...
mat13 + mat14 + mat15 + mat16 + mat17 + mat18 + mat19 + mat20 + mat21 + mat22 + mat23 + mat24 + mat25 + mat26 + mat27 + mat28)/28; %12 for orient track Kraft Paper eq.68


end

