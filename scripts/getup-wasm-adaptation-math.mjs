// Immutable 800 Hz interval arithmetic across arbitrary40 Hz packet boundaries.
export function intervalSummary(rows) {
  let longest=0,tail=0,prefix=0,leading=true;
  for(const row of rows) {
    const p=row.current.predicates;
    const pass=p.torso&&p.bothFeetLoadedAndPersistent&&p.footLoadFraction&&p.onlyFeet&&p.linearMotion&&p.angularMotion;
    if(pass){tail++;if(leading)prefix++;longest=Math.max(longest,tail);}
    else {tail=0;leading=false;}
  }
  return {longest,tail,prefix,count:rows.length};
}
export function appendInterval(previous,rows) {
  const packet=intervalSummary(rows);
  return {longest:Math.max(previous.longest,packet.longest,previous.tail+packet.prefix),
    tail:packet.tail===packet.count?previous.tail+packet.count:packet.tail};
}
