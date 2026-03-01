FROM node:20-alpine

WORKDIR /app

COPY web/package.json ./
RUN npm install --omit=dev

COPY web/server.js ./server.js
COPY web/public ./public

EXPOSE 8080

CMD ["npm", "start"]

