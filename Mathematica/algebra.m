(* Created with the Wolfram Language : www.wolfram.com *)
{{{{Sin[Pi/6 + (k*Pi)/3], Cos[Pi/6 + (k*Pi)/3], 0, 
    (d0*Sin[Pi/6 + (k*Pi)/3])/2}, {-Cos[Pi/6 + (k*Pi)/3], 
    Sin[Pi/6 + (k*Pi)/3], 0, -(d0*Cos[Pi/6 + (k*Pi)/3])/2}, {0, 0, 1, h0}, 
   {0, 0, 0, 1}}, {{Cos[Pi/6 + (k*Pi)/3]*Sin[a] + 
     Cos[a]*Sin[Pi/6 + (k*Pi)/3], Cos[a]*Cos[Pi/6 + (k*Pi)/3] - 
     Sin[a]*Sin[Pi/6 + (k*Pi)/3], 0, l1*Cos[Pi/6 + (k*Pi)/3]*Sin[a] + 
     (d0*Sin[Pi/6 + (k*Pi)/3])/2 + l1*Cos[a]*Sin[Pi/6 + (k*Pi)/3]}, 
   {-(Cos[a]*Cos[Pi/6 + (k*Pi)/3]) + Sin[a]*Sin[Pi/6 + (k*Pi)/3], 
    Cos[Pi/6 + (k*Pi)/3]*Sin[a] + Cos[a]*Sin[Pi/6 + (k*Pi)/3], 0, 
    -(d0*Cos[Pi/6 + (k*Pi)/3])/2 - l1*Cos[a]*Cos[Pi/6 + (k*Pi)/3] + 
     l1*Sin[a]*Sin[Pi/6 + (k*Pi)/3]}, {0, 0, 1, h0}, {0, 0, 0, 1}}, 
  {{Cos[b]*(Cos[Pi/6 + (k*Pi)/3]*Sin[a] + Cos[a]*Sin[Pi/6 + (k*Pi)/3]) + 
     Sin[b]*(Cos[a]*Cos[Pi/6 + (k*Pi)/3] - Sin[a]*Sin[Pi/6 + (k*Pi)/3]), 0, 
    -(Sin[b]*(Cos[Pi/6 + (k*Pi)/3]*Sin[a] + Cos[a]*Sin[Pi/6 + (k*Pi)/3])) + 
     Cos[b]*(Cos[a]*Cos[Pi/6 + (k*Pi)/3] - Sin[a]*Sin[Pi/6 + (k*Pi)/3]), 
    l1*Cos[Pi/6 + (k*Pi)/3]*Sin[a] + (d0*Sin[Pi/6 + (k*Pi)/3])/2 + 
     l1*Cos[a]*Sin[Pi/6 + (k*Pi)/3] + l2*Cos[b]*
      (Cos[Pi/6 + (k*Pi)/3]*Sin[a] + Cos[a]*Sin[Pi/6 + (k*Pi)/3]) + 
     l2*Sin[b]*(Cos[a]*Cos[Pi/6 + (k*Pi)/3] - Sin[a]*Sin[Pi/6 + (k*Pi)/3])}, 
   {Sin[b]*(Cos[Pi/6 + (k*Pi)/3]*Sin[a] + Cos[a]*Sin[Pi/6 + (k*Pi)/3]) + 
     Cos[b]*(-(Cos[a]*Cos[Pi/6 + (k*Pi)/3]) + Sin[a]*Sin[Pi/6 + (k*Pi)/3]), 
    0, Cos[b]*(Cos[Pi/6 + (k*Pi)/3]*Sin[a] + Cos[a]*Sin[Pi/6 + (k*Pi)/3]) - 
     Sin[b]*(-(Cos[a]*Cos[Pi/6 + (k*Pi)/3]) + Sin[a]*Sin[Pi/6 + (k*Pi)/3]), 
    -(d0*Cos[Pi/6 + (k*Pi)/3])/2 - l1*Cos[a]*Cos[Pi/6 + (k*Pi)/3] + 
     l1*Sin[a]*Sin[Pi/6 + (k*Pi)/3] + l2*Sin[b]*
      (Cos[Pi/6 + (k*Pi)/3]*Sin[a] + Cos[a]*Sin[Pi/6 + (k*Pi)/3]) + 
     l2*Cos[b]*(-(Cos[a]*Cos[Pi/6 + (k*Pi)/3]) + 
       Sin[a]*Sin[Pi/6 + (k*Pi)/3])}, {0, -1, 0, h0 + h2}, {0, 0, 0, 1}}, 
  {{Cos[c]*(Cos[b]*(Cos[Pi/6 + (k*Pi)/3]*Sin[a] + 
        Cos[a]*Sin[Pi/6 + (k*Pi)/3]) + Sin[b]*(Cos[a]*Cos[Pi/6 + (k*Pi)/3] - 
        Sin[a]*Sin[Pi/6 + (k*Pi)/3])), 
    -(Sin[c]*(Cos[b]*(Cos[Pi/6 + (k*Pi)/3]*Sin[a] + 
         Cos[a]*Sin[Pi/6 + (k*Pi)/3]) + Sin[b]*(Cos[a]*Cos[Pi/6 + (k*Pi)/3] - 
         Sin[a]*Sin[Pi/6 + (k*Pi)/3]))), 
    -(Sin[b]*(Cos[Pi/6 + (k*Pi)/3]*Sin[a] + Cos[a]*Sin[Pi/6 + (k*Pi)/3])) + 
     Cos[b]*(Cos[a]*Cos[Pi/6 + (k*Pi)/3] - Sin[a]*Sin[Pi/6 + (k*Pi)/3]), 
    l1*Cos[Pi/6 + (k*Pi)/3]*Sin[a] + (d0*Sin[Pi/6 + (k*Pi)/3])/2 + 
     l1*Cos[a]*Sin[Pi/6 + (k*Pi)/3] + l2*Cos[b]*
      (Cos[Pi/6 + (k*Pi)/3]*Sin[a] + Cos[a]*Sin[Pi/6 + (k*Pi)/3]) + 
     l2*Sin[b]*(Cos[a]*Cos[Pi/6 + (k*Pi)/3] - Sin[a]*Sin[Pi/6 + (k*Pi)/3]) + 
     h3*(-(Sin[b]*(Cos[Pi/6 + (k*Pi)/3]*Sin[a] + 
          Cos[a]*Sin[Pi/6 + (k*Pi)/3])) + 
       Cos[b]*(Cos[a]*Cos[Pi/6 + (k*Pi)/3] - Sin[a]*Sin[Pi/6 + (k*Pi)/3])) + 
     l3*Cos[c]*(Cos[b]*(Cos[Pi/6 + (k*Pi)/3]*Sin[a] + 
         Cos[a]*Sin[Pi/6 + (k*Pi)/3]) + Sin[b]*(Cos[a]*Cos[Pi/6 + (k*Pi)/3] - 
         Sin[a]*Sin[Pi/6 + (k*Pi)/3]))}, 
   {Cos[c]*(Sin[b]*(Cos[Pi/6 + (k*Pi)/3]*Sin[a] + 
        Cos[a]*Sin[Pi/6 + (k*Pi)/3]) + 
      Cos[b]*(-(Cos[a]*Cos[Pi/6 + (k*Pi)/3]) + Sin[a]*Sin[Pi/6 + (k*Pi)/3])), 
    -(Sin[c]*(Sin[b]*(Cos[Pi/6 + (k*Pi)/3]*Sin[a] + 
         Cos[a]*Sin[Pi/6 + (k*Pi)/3]) + 
       Cos[b]*(-(Cos[a]*Cos[Pi/6 + (k*Pi)/3]) + 
         Sin[a]*Sin[Pi/6 + (k*Pi)/3]))), 
    Cos[b]*(Cos[Pi/6 + (k*Pi)/3]*Sin[a] + Cos[a]*Sin[Pi/6 + (k*Pi)/3]) - 
     Sin[b]*(-(Cos[a]*Cos[Pi/6 + (k*Pi)/3]) + Sin[a]*Sin[Pi/6 + (k*Pi)/3]), 
    -(d0*Cos[Pi/6 + (k*Pi)/3])/2 - l1*Cos[a]*Cos[Pi/6 + (k*Pi)/3] + 
     l1*Sin[a]*Sin[Pi/6 + (k*Pi)/3] + l2*Sin[b]*
      (Cos[Pi/6 + (k*Pi)/3]*Sin[a] + Cos[a]*Sin[Pi/6 + (k*Pi)/3]) + 
     l2*Cos[b]*(-(Cos[a]*Cos[Pi/6 + (k*Pi)/3]) + 
       Sin[a]*Sin[Pi/6 + (k*Pi)/3]) + l3*Cos[c]*
      (Sin[b]*(Cos[Pi/6 + (k*Pi)/3]*Sin[a] + Cos[a]*Sin[Pi/6 + (k*Pi)/3]) + 
       Cos[b]*(-(Cos[a]*Cos[Pi/6 + (k*Pi)/3]) + 
         Sin[a]*Sin[Pi/6 + (k*Pi)/3])) + 
     h3*(Cos[b]*(Cos[Pi/6 + (k*Pi)/3]*Sin[a] + Cos[a]*Sin[Pi/6 + (k*Pi)/3]) - 
       Sin[b]*(-(Cos[a]*Cos[Pi/6 + (k*Pi)/3]) + 
         Sin[a]*Sin[Pi/6 + (k*Pi)/3]))}, {-Sin[c], -Cos[c], 0, 
    h0 + h2 - l3*Sin[c]}, {0, 0, 0, 1}}, 
  {{Cos[c + d]*Sin[a + b + (Pi + 2*k*Pi)/6], 
    -(Sin[c + d]*Sin[a + b + (Pi + 2*k*Pi)/6]), Cos[a + b + (Pi + 2*k*Pi)/6], 
    h3*Cos[a + b + ((1 + 2*k)*Pi)/6] + (d0*Sin[(Pi + 2*k*Pi)/6])/2 + 
     (l2 + l3*Cos[c] + l4*Cos[c + d])*Sin[a + b + ((1 + 2*k)*Pi)/6] + 
     l1*Sin[a + (Pi + 2*k*Pi)/6]}, 
   {-(Cos[c + d]*Cos[a + b + (Pi + 2*k*Pi)/6]), Cos[a + b + (Pi + 2*k*Pi)/6]*
     Sin[c + d], Sin[a + b + (Pi + 2*k*Pi)/6], -(d0*Cos[(Pi + 2*k*Pi)/6])/2 - 
     l1*Cos[a + (Pi + 2*k*Pi)/6] - (l2 + l3*Cos[c] + l4*Cos[c + d])*
      Cos[a + b + (Pi + 2*k*Pi)/6] + h3*Sin[a + b + (Pi + 2*k*Pi)/6]}, 
   {-Sin[c + d], -Cos[c + d], 0, h0 + h2 - l3*Sin[c] - l4*Sin[c + d]}, 
   {0, 0, 0, 1}}, 
  {{-(Sin[alpha + beta + c + d]*Sin[a + b + ((1 + 2*k)*Pi)/6]), 
    Cos[a + b + ((1 + 2*k)*Pi)/6], Cos[alpha + beta + c + d]*
     Sin[a + b + ((1 + 2*k)*Pi)/6], h3*Cos[a + b + ((1 + 2*k)*Pi)/6] + 
     (d0*Sin[(Pi + 2*k*Pi)/6])/2 + l1*Sin[a + ((1 + 2*k)*Pi)/6] + 
     (l2 + l3*Cos[c] + xinf*Cos[c + d] + r1*Cos[alpha + c + d] - 
       yinf*Sin[c + d])*Sin[a + b + ((1 + 2*k)*Pi)/6]}, 
   {Cos[a + b + ((1 + 2*k)*Pi)/6]*Sin[alpha + beta + c + d], 
    Sin[a + b + ((1 + 2*k)*Pi)/6], -(Cos[alpha + beta + c + d]*
      Cos[a + b + ((1 + 2*k)*Pi)/6]), -(d0*Cos[(Pi + 2*k*Pi)/6])/2 - 
     l1*Cos[a + ((1 + 2*k)*Pi)/6] - Cos[a + b + ((1 + 2*k)*Pi)/6]*
      (l2 + l3*Cos[c] + xinf*Cos[c + d] + r1*Cos[alpha + c + d] - 
       yinf*Sin[c + d]) + h3*Sin[a + b + ((1 + 2*k)*Pi)/6]}, 
   {-Cos[alpha + beta + c + d], 0, -Sin[alpha + beta + c + d], 
    h0 + h2 - yinf*Cos[c + d] - l3*Sin[c] - xinf*Sin[c + d] - 
     r1*Sin[alpha + c + d]}, {0, 0, 0, 1}}}, 
 {{{Sin[gamma + Pi/6 + (k*Pi)/3], 0, -Cos[gamma + Pi/6 + (k*Pi)/3], 
    (d1*Sin[gamma + Pi/6 + (k*Pi)/3])/2}, {-Cos[gamma + Pi/6 + (k*Pi)/3], 0, 
    -Sin[gamma + Pi/6 + (k*Pi)/3], -(d1*Cos[gamma + Pi/6 + (k*Pi)/3])/2}, 
   {0, 1, 0, h1}, {0, 0, 0, 1}}, {{Cos[a]*Sin[gamma + Pi/6 + (k*Pi)/3], 
    -(Sin[a]*Sin[gamma + Pi/6 + (k*Pi)/3]), -Cos[gamma + Pi/6 + (k*Pi)/3], 
    (d1*Sin[gamma + Pi/6 + (k*Pi)/3])/2 + 
     ls1*Cos[a]*Sin[gamma + Pi/6 + (k*Pi)/3]}, 
   {-(Cos[a]*Cos[gamma + Pi/6 + (k*Pi)/3]), Cos[gamma + Pi/6 + (k*Pi)/3]*
     Sin[a], -Sin[gamma + Pi/6 + (k*Pi)/3], 
    -(d1*Cos[gamma + Pi/6 + (k*Pi)/3])/2 - 
     ls1*Cos[a]*Cos[gamma + Pi/6 + (k*Pi)/3]}, {Sin[a], Cos[a], 0, 
    h1 + ls1*Sin[a]}, {0, 0, 0, 1}}, 
  {{Cos[a + b]*Sin[gamma + (Pi + 2*k*Pi)/6], 
    -(Sin[a + b]*Sin[gamma + (Pi + 2*k*Pi)/6]), 
    -Cos[gamma + (Pi + 2*k*Pi)/6], ((d1 + 2*ls1*Cos[a] + 2*ls2*Cos[a + b])*
      Sin[gamma + (Pi + 2*k*Pi)/6])/2}, 
   {-(Cos[a + b]*Cos[gamma + (Pi + 2*k*Pi)/6]), Cos[gamma + (Pi + 2*k*Pi)/6]*
     Sin[a + b], -Sin[gamma + (Pi + 2*k*Pi)/6], 
    -((d1 + 2*ls1*Cos[a] + 2*ls2*Cos[a + b])*Cos[gamma + (Pi + 2*k*Pi)/6])/
     2}, {Sin[a + b], Cos[a + b], 0, h1 + ls1*Sin[a] + ls2*Sin[a + b]}, 
   {0, 0, 0, 1}}, 
  {{-(Sin[a + b + delta + epsilon]*Sin[gamma + (Pi + 2*k*Pi)/6]), 
    Cos[gamma + (Pi + 2*k*Pi)/6], -(Cos[a + b + delta + epsilon]*
      Sin[gamma + (Pi + 2*k*Pi)/6]), 
    ((d1 + 2*ls1*Cos[a] + 2*xsup*Cos[a + b] + 2*r2*Cos[a + b + delta] - 
       2*ysup*Sin[a + b])*Sin[gamma + (Pi + 2*k*Pi)/6])/2}, 
   {Cos[gamma + (Pi + 2*k*Pi)/6]*Sin[a + b + delta + epsilon], 
    Sin[gamma + (Pi + 2*k*Pi)/6], Cos[a + b + delta + epsilon]*
     Cos[gamma + (Pi + 2*k*Pi)/6], 
    -(Cos[gamma + (Pi + 2*k*Pi)/6]*(d1 + 2*ls1*Cos[a] + 2*xsup*Cos[a + b] + 
        2*r2*Cos[a + b + delta] - 2*ysup*Sin[a + b]))/2}, 
   {Cos[a + b + delta + epsilon], 0, -Sin[a + b + delta + epsilon], 
    h1 + ysup*Cos[a + b] + ls1*Sin[a] + xsup*Sin[a + b] + 
     r2*Sin[a + b + delta]}, {0, 0, 0, 1}}}}
