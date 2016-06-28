function M = polydiffcfs( d )
%
% Purpose : Generate the matrix M to compute the derivative of the
% coefficients of a polynomial, i.e.
%   y = V * p and dy/dx = V * M * p
%
% Use (syntax):
%   M = polydiffcfs( d )
%
% Input Parameters :
%   d: the degree of the polynomial
%
% Return Parameters :
%   M: the Derivative matrix for the coefficients
%
% Description and algorithms:
%
% References : 
%
% Author :  Matthew Harker and Paul O'Leary
% Date :    17. January 2013
% Version : 1.0
%
% (c) 2013 Matthew Harker and Paul O'Leary, 
% Chair of Automation, University of Leoben, Leoben, Austria
% email: office@harkeroleary.org, 
% url: www.harkeroleary.org
%
% History:
%   Date:           Comment:

%
M = zeros( d + 1 );
%
for k=1:d
    M(k+1,k) = d - k + 1;
end;
