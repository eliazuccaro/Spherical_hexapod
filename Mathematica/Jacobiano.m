(* Created with the Wolfram Language : www.wolfram.com *)
{{{{(l2 + l3*Cos[c] + l4*Cos[c + d])*Cos[a + b + ((1 + 2*k)*Pi)/6] + 
     l1*Cos[a + (Pi + 2*k*Pi)/6] - h3*Sin[a + b + ((1 + 2*k)*Pi)/6], 
    (l2 + l3*Cos[c] + l4*Cos[c + d])*Cos[a + b + ((1 + 2*k)*Pi)/6] - 
     h3*Sin[a + b + ((1 + 2*k)*Pi)/6], 
    -((l3*Sin[c] + l4*Sin[c + d])*Sin[a + b + ((1 + 2*k)*Pi)/6]), 
    -(l4*Sin[c + d]*Sin[a + b + ((1 + 2*k)*Pi)/6])}, 
   {h3*Cos[a + b + (Pi + 2*k*Pi)/6] + l1*Sin[a + (Pi + 2*k*Pi)/6] + 
     (l2 + l3*Cos[c] + l4*Cos[c + d])*Sin[a + b + (Pi + 2*k*Pi)/6], 
    h3*Cos[a + b + (Pi + 2*k*Pi)/6] + (l2 + l3*Cos[c] + l4*Cos[c + d])*
      Sin[a + b + (Pi + 2*k*Pi)/6], Cos[a + b + (Pi + 2*k*Pi)/6]*
     (l3*Sin[c] + l4*Sin[c + d]), l4*Cos[a + b + (Pi + 2*k*Pi)/6]*
     Sin[c + d]}, {0, 0, -(l3*Cos[c]) - l4*Cos[c + d], -(l4*Cos[c + d])}, 
   {1, 1, 0, 0}, {0, 0, 1, 1}, {0, 0, 0, 0}}, 
  {{(l2 + l3*Cos[b] + l4*Cos[b + c])*Cos[a + ((1 + 2*k)*Pi)/6] - 
     h3*Sin[a + ((1 + 2*k)*Pi)/6], -((l3*Sin[b] + l4*Sin[b + c])*
      Sin[a + ((1 + 2*k)*Pi)/6]), 
    -(l4*Sin[b + c]*Sin[a + ((1 + 2*k)*Pi)/6])}, 
   {h3*Cos[a + (Pi + 2*k*Pi)/6] + (l2 + l3*Cos[b] + l4*Cos[b + c])*
      Sin[a + (Pi + 2*k*Pi)/6], Cos[a + (Pi + 2*k*Pi)/6]*
     (l3*Sin[b] + l4*Sin[b + c]), l4*Cos[a + (Pi + 2*k*Pi)/6]*Sin[b + c]}, 
   {0, -(l3*Cos[b]) - l4*Cos[b + c], -(l4*Cos[b + c])}}, 
  {{l1*Cos[a + (Pi + 2*k*Pi)/6] + Cos[a + b + ((1 + 2*k)*Pi)/6]*
      (l2 + l3*Cos[c] + xinf*Cos[c + d] + r1*Cos[alpha + c + d] - 
       yinf*Sin[c + d]) - h3*Sin[a + b + ((1 + 2*k)*Pi)/6], 
    Cos[a + b + ((1 + 2*k)*Pi)/6]*(l2 + l3*Cos[c] + xinf*Cos[c + d] + 
       r1*Cos[alpha + c + d] - yinf*Sin[c + d]) - 
     h3*Sin[a + b + ((1 + 2*k)*Pi)/6], 
    -((yinf*Cos[c + d] + l3*Sin[c] + xinf*Sin[c + d] + r1*Sin[alpha + c + d])*
      Sin[a + b + ((1 + 2*k)*Pi)/6]), 
    -((yinf*Cos[c + d] + xinf*Sin[c + d] + r1*Sin[alpha + c + d])*
      Sin[a + b + ((1 + 2*k)*Pi)/6])}, 
   {h3*Cos[a + b + ((1 + 2*k)*Pi)/6] + l1*Sin[a + ((1 + 2*k)*Pi)/6] + 
     (l2 + l3*Cos[c] + xinf*Cos[c + d] + r1*Cos[alpha + c + d] - 
       yinf*Sin[c + d])*Sin[a + b + ((1 + 2*k)*Pi)/6], 
    h3*Cos[a + b + ((1 + 2*k)*Pi)/6] + (l2 + l3*Cos[c] + xinf*Cos[c + d] + 
       r1*Cos[alpha + c + d] - yinf*Sin[c + d])*
      Sin[a + b + ((1 + 2*k)*Pi)/6], Cos[a + b + ((1 + 2*k)*Pi)/6]*
     (yinf*Cos[c + d] + l3*Sin[c] + xinf*Sin[c + d] + r1*Sin[alpha + c + d]), 
    Cos[a + b + ((1 + 2*k)*Pi)/6]*(yinf*Cos[c + d] + xinf*Sin[c + d] + 
      r1*Sin[alpha + c + d])}, {0, 0, -(l3*Cos[c]) - xinf*Cos[c + d] - 
     r1*Cos[alpha + c + d] + yinf*Sin[c + d], -(xinf*Cos[c + d]) - 
     r1*Cos[alpha + c + d] + yinf*Sin[c + d]}, {1, 1, 0, 0}, {0, 0, 1, 1}, 
   {0, 0, 0, 0}}, 
  {{((-2*ls1*Sin[a] - 2*ls2*Sin[a + b])*Sin[gamma + (Pi + 2*k*Pi)/6])/2, 
    -(ls2*Sin[a + b]*Sin[gamma + (Pi + 2*k*Pi)/6])}, 
   {-(Cos[gamma + (Pi + 2*k*Pi)/6]*(-2*ls1*Sin[a] - 2*ls2*Sin[a + b]))/2, 
    ls2*Cos[gamma + (Pi + 2*k*Pi)/6]*Sin[a + b]}, 
   {ls1*Cos[a] + ls2*Cos[a + b], ls2*Cos[a + b]}, {0, 0}, {1, 1}, {0, 0}}, 
  {{((-2*ysup*Cos[a + b] - 2*ls1*Sin[a] - 2*xsup*Sin[a + b] - 
       2*r2*Sin[a + b + delta])*Sin[gamma + (Pi + 2*k*Pi)/6])/2, 
    ((-2*ysup*Cos[a + b] - 2*xsup*Sin[a + b] - 2*r2*Sin[a + b + delta])*
      Sin[gamma + (Pi + 2*k*Pi)/6])/2}, 
   {-(Cos[gamma + (Pi + 2*k*Pi)/6]*(-2*ysup*Cos[a + b] - 2*ls1*Sin[a] - 
        2*xsup*Sin[a + b] - 2*r2*Sin[a + b + delta]))/2, 
    -(Cos[gamma + (Pi + 2*k*Pi)/6]*(-2*ysup*Cos[a + b] - 2*xsup*Sin[a + b] - 
        2*r2*Sin[a + b + delta]))/2}, 
   {ls1*Cos[a] + xsup*Cos[a + b] + r2*Cos[a + b + delta] - ysup*Sin[a + b], 
    xsup*Cos[a + b] + r2*Cos[a + b + delta] - ysup*Sin[a + b]}, {0, 0}, 
   {1, 1}, {0, 0}}}, 
 {{{(l2 + l3*Cos[c] + l4*Cos[c + d])*Cos[a + b + ((1 + 2*k)*Pi)/6] + 
     l1*Cos[a + (Pi + 2*k*Pi)/6] - h3*Sin[a + b + ((1 + 2*k)*Pi)/6], 
    (l2 + l3*Cos[c] + l4*Cos[c + d])*Cos[a + b + ((1 + 2*k)*Pi)/6] - 
     h3*Sin[a + b + ((1 + 2*k)*Pi)/6], 
    -((l3*Sin[c] + l4*Sin[c + d])*Sin[a + b + ((1 + 2*k)*Pi)/6]), 
    -(l4*Sin[c + d]*Sin[a + b + ((1 + 2*k)*Pi)/6])}, 
   {h3*Cos[a + b + (Pi + 2*k*Pi)/6] + l1*Sin[a + (Pi + 2*k*Pi)/6] + 
     (l2 + l3*Cos[c] + l4*Cos[c + d])*Sin[a + b + (Pi + 2*k*Pi)/6], 
    h3*Cos[a + b + (Pi + 2*k*Pi)/6] + (l2 + l3*Cos[c] + l4*Cos[c + d])*
      Sin[a + b + (Pi + 2*k*Pi)/6], Cos[a + b + (Pi + 2*k*Pi)/6]*
     (l3*Sin[c] + l4*Sin[c + d]), l4*Cos[a + b + (Pi + 2*k*Pi)/6]*
     Sin[c + d]}, {0, 0, -(l3*Cos[c]) - l4*Cos[c + d], -(l4*Cos[c + d])}, 
   {0, 0, Cos[a + b + Pi/6 + (k*Pi)/3], Cos[a + b + Pi/6 + (k*Pi)/3]}, 
   {0, 0, Sin[a + b + Pi/6 + (k*Pi)/3], Sin[a + b + Pi/6 + (k*Pi)/3]}, 
   {1, 1, 0, 0}}, {{l1*Cos[a + (Pi + 2*k*Pi)/6] + 
     Cos[a + b + ((1 + 2*k)*Pi)/6]*(l2 + l3*Cos[c] + xinf*Cos[c + d] + 
       r1*Cos[alpha + c + d] - yinf*Sin[c + d]) - 
     h3*Sin[a + b + ((1 + 2*k)*Pi)/6], 
    Cos[a + b + ((1 + 2*k)*Pi)/6]*(l2 + l3*Cos[c] + xinf*Cos[c + d] + 
       r1*Cos[alpha + c + d] - yinf*Sin[c + d]) - 
     h3*Sin[a + b + ((1 + 2*k)*Pi)/6], 
    -((yinf*Cos[c + d] + l3*Sin[c] + xinf*Sin[c + d] + r1*Sin[alpha + c + d])*
      Sin[a + b + ((1 + 2*k)*Pi)/6]), 
    -((yinf*Cos[c + d] + xinf*Sin[c + d] + r1*Sin[alpha + c + d])*
      Sin[a + b + ((1 + 2*k)*Pi)/6])}, 
   {h3*Cos[a + b + ((1 + 2*k)*Pi)/6] + l1*Sin[a + ((1 + 2*k)*Pi)/6] + 
     (l2 + l3*Cos[c] + xinf*Cos[c + d] + r1*Cos[alpha + c + d] - 
       yinf*Sin[c + d])*Sin[a + b + ((1 + 2*k)*Pi)/6], 
    h3*Cos[a + b + ((1 + 2*k)*Pi)/6] + (l2 + l3*Cos[c] + xinf*Cos[c + d] + 
       r1*Cos[alpha + c + d] - yinf*Sin[c + d])*
      Sin[a + b + ((1 + 2*k)*Pi)/6], Cos[a + b + ((1 + 2*k)*Pi)/6]*
     (yinf*Cos[c + d] + l3*Sin[c] + xinf*Sin[c + d] + r1*Sin[alpha + c + d]), 
    Cos[a + b + ((1 + 2*k)*Pi)/6]*(yinf*Cos[c + d] + xinf*Sin[c + d] + 
      r1*Sin[alpha + c + d])}, {0, 0, -(l3*Cos[c]) - xinf*Cos[c + d] - 
     r1*Cos[alpha + c + d] + yinf*Sin[c + d], -(xinf*Cos[c + d]) - 
     r1*Cos[alpha + c + d] + yinf*Sin[c + d]}, 
   {0, 0, Cos[a + b + Pi/6 + (k*Pi)/3], Cos[a + b + Pi/6 + (k*Pi)/3]}, 
   {0, 0, Sin[a + b + Pi/6 + (k*Pi)/3], Sin[a + b + Pi/6 + (k*Pi)/3]}, 
   {1, 1, 0, 0}}, 
  {{((-2*ls1*Sin[a] - 2*ls2*Sin[a + b])*Sin[gamma + (Pi + 2*k*Pi)/6])/2, 
    -(ls2*Sin[a + b]*Sin[gamma + (Pi + 2*k*Pi)/6])}, 
   {-(Cos[gamma + (Pi + 2*k*Pi)/6]*(-2*ls1*Sin[a] - 2*ls2*Sin[a + b]))/2, 
    ls2*Cos[gamma + (Pi + 2*k*Pi)/6]*Sin[a + b]}, 
   {ls1*Cos[a] + ls2*Cos[a + b], ls2*Cos[a + b]}, 
   {Cos[gamma + Pi/6 + (k*Pi)/3], Cos[gamma + Pi/6 + (k*Pi)/3]}, 
   {Sin[gamma + Pi/6 + (k*Pi)/3], Sin[gamma + Pi/6 + (k*Pi)/3]}, {0, 0}}, 
  {{((-2*ysup*Cos[a + b] - 2*ls1*Sin[a] - 2*xsup*Sin[a + b] - 
       2*r2*Sin[a + b + delta])*Sin[gamma + (Pi + 2*k*Pi)/6])/2, 
    ((-2*ysup*Cos[a + b] - 2*xsup*Sin[a + b] - 2*r2*Sin[a + b + delta])*
      Sin[gamma + (Pi + 2*k*Pi)/6])/2}, 
   {-(Cos[gamma + (Pi + 2*k*Pi)/6]*(-2*ysup*Cos[a + b] - 2*ls1*Sin[a] - 
        2*xsup*Sin[a + b] - 2*r2*Sin[a + b + delta]))/2, 
    -(Cos[gamma + (Pi + 2*k*Pi)/6]*(-2*ysup*Cos[a + b] - 2*xsup*Sin[a + b] - 
        2*r2*Sin[a + b + delta]))/2}, 
   {ls1*Cos[a] + xsup*Cos[a + b] + r2*Cos[a + b + delta] - ysup*Sin[a + b], 
    xsup*Cos[a + b] + r2*Cos[a + b + delta] - ysup*Sin[a + b]}, 
   {Cos[gamma + Pi/6 + (k*Pi)/3], Cos[gamma + Pi/6 + (k*Pi)/3]}, 
   {Sin[gamma + Pi/6 + (k*Pi)/3], Sin[gamma + Pi/6 + (k*Pi)/3]}, {0, 0}}}, 
 {{h3*Cos[a + b + ((1 + 2*k)*Pi)/6] + (d0*Sin[(Pi + 2*k*Pi)/6])/2 + 
    (l2 + l3*Cos[c] + l4*Cos[c + d])*Sin[a + b + ((1 + 2*k)*Pi)/6] + 
    l1*Sin[a + (Pi + 2*k*Pi)/6], -(d0*Cos[(Pi + 2*k*Pi)/6])/2 - 
    l1*Cos[a + (Pi + 2*k*Pi)/6] - (l2 + l3*Cos[c] + l4*Cos[c + d])*
     Cos[a + b + (Pi + 2*k*Pi)/6] + h3*Sin[a + b + (Pi + 2*k*Pi)/6], 
   h0 + h2 - l3*Sin[c] - l4*Sin[c + d], a + b - Pi/3 + (k*Pi)/3, c + d, 0}, 
  {h3*Cos[a + b + ((1 + 2*k)*Pi)/6] + (d0*Sin[(Pi + 2*k*Pi)/6])/2 + 
    l1*Sin[a + ((1 + 2*k)*Pi)/6] + (l2 + l3*Cos[c] + xinf*Cos[c + d] + 
      r1*Cos[alpha + c + d] - yinf*Sin[c + d])*Sin[a + b + ((1 + 2*k)*Pi)/6], 
   -(d0*Cos[(Pi + 2*k*Pi)/6])/2 - l1*Cos[a + ((1 + 2*k)*Pi)/6] - 
    Cos[a + b + ((1 + 2*k)*Pi)/6]*(l2 + l3*Cos[c] + xinf*Cos[c + d] + 
      r1*Cos[alpha + c + d] - yinf*Sin[c + d]) + 
    h3*Sin[a + b + ((1 + 2*k)*Pi)/6], h0 + h2 - yinf*Cos[c + d] - l3*Sin[c] - 
    xinf*Sin[c + d] - r1*Sin[alpha + c + d], a + b - Pi/3 + (k*Pi)/3, 
   alpha + beta + c + d + Pi/2, 0}, 
  {((d1 + 2*ls1*Cos[a] + 2*ls2*Cos[a + b])*Sin[gamma + (Pi + 2*k*Pi)/6])/2, 
   -((d1 + 2*ls1*Cos[a] + 2*ls2*Cos[a + b])*Cos[gamma + (Pi + 2*k*Pi)/6])/2, 
   h1 + ls1*Sin[a] + ls2*Sin[a + b], gamma - Pi/3 + (k*Pi)/3, a + b, 0}, 
  {((d1 + 2*ls1*Cos[a] + 2*xsup*Cos[a + b] + 2*r2*Cos[a + b + delta] - 
      2*ysup*Sin[a + b])*Sin[gamma + (Pi + 2*k*Pi)/6])/2, 
   -(Cos[gamma + (Pi + 2*k*Pi)/6]*(d1 + 2*ls1*Cos[a] + 2*xsup*Cos[a + b] + 
       2*r2*Cos[a + b + delta] - 2*ysup*Sin[a + b]))/2, 
   h1 + ysup*Cos[a + b] + ls1*Sin[a] + xsup*Sin[a + b] + 
    r2*Sin[a + b + delta], gamma + ((-1 + k)*Pi)/3, a + b, 0}}}
