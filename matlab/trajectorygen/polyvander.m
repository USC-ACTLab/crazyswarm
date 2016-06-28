function V = polyvander( x, d, mu)
%
% Purpose : This function generates a Vandermonde matrix compatible with
% the polyfit and pilyval functions in MATLAB. This function is some times
% convenient when computing covariance propagation for more involved
% problems.
%
% Use (syntax):
%      V = polyvander( x, d );
% or   V = polyvander( x, d, mu );
%
% Input Parameters :
%   x: the x vector for which the matrix is to be computed
%   d: maximum degree of the polynomial basis.
%   mu: (optional) mu(1) is the mean x value and mu(2) the standard
%   deviation. This is required to ensure complete campatability with
%   polyfit and polyval.
%
% Return Parameters :
%   V: The Vandermonde matrix
%
% Description and algorithms:
%
% References : 
%
% Author :  Matther Harker and Paul O'Leary
% Date :    20. April 2013
% Version : 1.0
%
% (c) 2013 Matther Harker and Paul O'Leary
% url: www.harkeroleary.org
% email: office@harkeroleary.org
%
% History:
%   Date:           Comment:
%

%--------------------------------------------
% Paramater testing
%--------------------------------------------
% test is x is a column vector
[n,m] = size(x);
if ~((n > 0)&&(m==1))
    error('The vector x must be a column vector.');
end;
%--------------------------------------------
% Shift and scale the x vector if required
%
if nargin > 2
    x = (x - mu(1))/mu(2);
end;
%
% Use Horner form to generate the Vandermonde matrix.
%
V = ones( length(x), d + 1);
V(:,d) = x;
for k=(d-1):-1:1
    V(:,k) = x.*V(:,k+1);
end;